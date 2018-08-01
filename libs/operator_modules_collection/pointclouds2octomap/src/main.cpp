/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <mapit/operators/module.h>
#include <mapit/logging.h>
#include <mapit/layertypes/pointcloudlayer.h>
#include <mapit/layertypes/octomaplayer.h>
#include <mapit/layertypes/tflayer/tf2/buffer_core.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <memory>
#include <mapit/errorcodes.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/depthfirstsearch.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <pcl/io/pcd_io.h>

void
addPointcloud(  mapit::OperationEnvironment* env
              , std::string const &tf_prefix
              , std::string const &input_name
              , std::string const &target_frame_id
              , std::shared_ptr<octomap::OcTree>& targetOctomap
              , std::string& frame_id
              , mapit::time::Stamp& stamp )
{
    // get entity
    std::shared_ptr<mapit::msgs::Entity> input_entity = env->getWorkspace()->getEntity(input_name);
    if ( input_entity == nullptr ) {
        log_error("operator pointclouds2octomap: entity \"" + input_name + "\" dosn't exists");
        throw MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    if ( input_entity->type() != PointcloudEntitydata::TYPENAME()) {
        log_error("operator pointclouds2octomap: entity \"" + input_name + "\" is of wrong type " + input_entity->type());
        throw MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    // set stamp and frame_id
    stamp = mapit::time::from_msg( input_entity->stamp() );
    frame_id = input_entity->frame_id();

    // get entity data
    std::shared_ptr<mapit::AbstractEntitydata> abstract_input = env->getWorkspace()->getEntitydataForReadWrite( input_name );
    assert(abstract_input);
    std::shared_ptr<PointcloudEntitydata> input_entity_data = std::static_pointer_cast<PointcloudEntitydata>( abstract_input );
    assert(input_entity_data);

    std::shared_ptr<pcl::PCLPointCloud2> input_pc2 = input_entity_data->getData();
    pcl::PointCloud<pcl::PointXYZ> input_pc;
    pcl::fromPCLPointCloud2(*input_pc2, input_pc);

    // get transform
    mapit::tf2::BufferCore buffer(env->getWorkspace(), tf_prefix);
    mapit::tf::TransformStamped tf = buffer.lookupTransform(target_frame_id, frame_id, stamp);

    // add cloud to octomap
    octomap::Pointcloud cloudOM;
    octomap::point3d identity;
    octomath::Vector3 pose_trans(  tf.transform.translation.x()
                                 , tf.transform.translation.y()
                                 , tf.transform.translation.z() );
    octomath::Quaternion pose_quat(  tf.transform.rotation.w()
                                   , tf.transform.rotation.x()
                                   , tf.transform.rotation.y()
                                   , tf.transform.rotation.z());
    octomap::pose6d pose(pose_trans, pose_quat);

    for (pcl::PointXYZ point : input_pc) {
        cloudOM.push_back( point.x, point.y, point.z );
    }
    targetOctomap->insertPointCloud(cloudOM, identity, pose);

}

std::vector<std::string>
get_param_input(mapit::OperationEnvironment* env, const QJsonObject& params)
{
    std::vector<std::string> cfg_input;
    std::list<std::string> input_list;
    if        ( params["input"].isString() ) {
        input_list.push_back( params["input"].toString().toStdString() );
    } else if ( params["input"].isArray() ) {
        for (QJsonValue input : params["input"].toArray() ) {
            if ( ! input.isString() ) {
                log_error("operator pointclouds2octomap: cfg \"input\" is not an array of strings");
                throw MAPIT_STATUS_INVALID_ARGUMENT;
            }
            input_list.push_back( input.toString().toStdString() );
        }
    } else {
        log_error("operator pointclouds2octomap: cfg \"input\" does not is a string or array of strings");
        throw MAPIT_STATUS_INVALID_ARGUMENT;
    }

    // search for all in input
    // if entity -> add to list
    // if tree -> depth search, add all entities to list
    // otherwise error
    for (std::string input_name : input_list) {
        if (env->getWorkspace()->getEntity(input_name) != nullptr) {
            cfg_input.push_back(input_name);
        } else {
            if (env->getWorkspace()->getTree(input_name) != nullptr) {
                env->getWorkspace()->depthFirstSearch(
                              input_name
                            , depthFirstSearchWorkspaceAll(mapit::msgs::Tree)
                            , depthFirstSearchWorkspaceAll(mapit::msgs::Tree)
                            , [&](std::shared_ptr<mapit::msgs::Entity> obj, const ObjectReference& ref, const mapit::Path &path)
                              {
                                  cfg_input.push_back(path);
                                  return true;
                              }
                            , depthFirstSearchWorkspaceAll(mapit::msgs::Entity)
                            );
            } else {
                log_error("RegistrationStorageHelper: pointcloud name \"" + input_name + "\" given in param \"input\", is neither a entity nor a tree");
                throw MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
            }
        }
    }

    return cfg_input;
}

mapit::StatusCode operate_pcds2octomap(mapit::OperationEnvironment* env)
{
    /** structure:
     * {
     *  <string>"input"[] : ..., // can either be, a list of entities or a tree containing enteties // must be in sensor frame
     *  <string>"target" : ...,
     *  <float>"resolution" : ...,
     *  <string>"frame_id" : ..., // registered frame
     *  <string>"tf-prefix" : ...
     * }
     */
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    log_info( "operator pointclouds2octomap: params:" + env->getParameters() );
    QJsonObject params(paramsDoc.object());

    // --cfg - input
    std::vector<std::string> input_names;
    try {
        input_names = get_param_input(env, params);
    } catch (mapit::StatusCode status) {
        return status;
    }

    // --cfg - target
    std::string target_name = params["target"].toString().toStdString();
    if (target_name.empty()) {
        log_error("operator pointclouds2octomap: cfg \"target\" is empty");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }

    std::string frame_id = params["frame_id"].toString().toStdString();
    if (frame_id.empty()) {
        log_error("operator pointclouds2octomap: cfg \"frame_id\" is empty");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    std::string tf_prefix = params["tf-prefix"].toString().toStdString();

    double resolution = params["resolution"].toDouble(0.0);
    if (resolution < 1e-5) {
        log_error("operator pointclouds2octomap: cfg \"resolution\" too small or empty, can't set to " << resolution << " should at least be 0.001");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }

    // create (or overwrite) target entity
    std::shared_ptr<mapit::msgs::Entity> target_entity = std::make_shared<mapit::msgs::Entity>();
    target_entity->set_type(mapit::entitytypes::Octomap::TYPENAME());
    env->getWorkspace()->storeEntity(target_name, target_entity);
    std::shared_ptr<mapit::AbstractEntitydata> abstractTarget = env->getWorkspace()->getEntitydataForReadWrite( target_name );
    std::shared_ptr<mapit::entitytypes::Octomap> targetEntityData = std::static_pointer_cast<mapit::entitytypes::Octomap>( abstractTarget );
    std::shared_ptr<octomap::OcTree> targetOctomap = std::make_shared<octomap::OcTree>(resolution);

    bool first_entity_processed = false;
    std::string frame_id_clouds;
    mapit::time::Stamp stamp_earliest;

    // add all pointclouds from source
    for (std::string input_name : input_names) {
        try {
            std::string frame_id_current;
            mapit::time::Stamp stamp_current;
            try {
                addPointcloud(env, tf_prefix, input_name, frame_id, targetOctomap, frame_id_current, stamp_current);
            } catch (...) {
                log_error("operator pointclouds2octomap: can't add entity " << input_name << " transform error");
                return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
            }

            if ( first_entity_processed ) {
                if ( frame_id_clouds != frame_id_current ) {
                    log_error("operator pointclouds2octomap: entities contain different frame_ids.\n"
                              "before was " + frame_id_clouds + ", but entity " + input_name + " has frame_id " + frame_id_current);
                    return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
                }
                if ( mapit::time::to_sec(stamp_current) < mapit::time::to_sec(stamp_earliest) ) {
                    stamp_earliest = stamp_current;
                }
            } else {
                first_entity_processed = true;
                frame_id_clouds = frame_id_current;
                stamp_earliest = stamp_current;
            }
        } catch (mapit::StatusCode status) {
            return status;
        }
    }

    // store target
    target_entity->set_frame_id( frame_id );
    target_entity->set_allocated_stamp( mapit::time::to_msg_allocated( stamp_earliest ) );
    env->getWorkspace()->storeEntity(target_name, target_entity);

    targetEntityData->setData(targetOctomap);

    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME, "construct a octomap from several pointclouds", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, true, &operate_pcds2octomap)
