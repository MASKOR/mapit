/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2016-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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
#include <mapit/layertypes/tflayer/tf2/buffer_core.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>
#include <mapit/depthfirstsearch.h>
#include <mapit/layertypes/octomaplayer.h>
#include <mapit/layertypes/grid2d.h>
#include <mapit/layertypes/grid2dHelper.h>
#include <iostream>
#include <memory>
#include <mapit/errorcodes.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

std::vector<mapit::time::Stamp>
get_param_input(mapit::OperationEnvironment* env, const QJsonObject& params)
{
    std::vector<mapit::time::Stamp> stamps;
    std::list<std::string> input_list;
    if        ( params["path-entity-stamps"].isString() ) {
        input_list.push_back( params["path-entity-stamps"].toString().toStdString() );
    } else if ( params["path-entity-stamps"].isArray() ) {
        for (QJsonValue input : params["path-entity-stamps"].toArray() ) {
            if ( ! input.isString() ) {
                log_error("operator octomap2occupancy_grid: cfg \"path-entity-stamps\" is not an array of strings");
                throw MAPIT_STATUS_INVALID_ARGUMENT;
            }
            input_list.push_back( input.toString().toStdString() );
        }
    } else {
        log_error("operator octomap2occupancy_grid: cfg \"path-entity-stamps\" does not is a string or array of strings");
        throw MAPIT_STATUS_INVALID_ARGUMENT;
    }

    // search for all in input
    // if entity -> add to list
    // if tree -> depth search, add all entities to list
    // otherwise error
    for (std::string input_name : input_list) {
        std::shared_ptr<mapit::msgs::Entity> entity = env->getWorkspace()->getEntity(input_name);
        if (entity != nullptr) {
            stamps.push_back( mapit::time::from_msg(entity->stamp()) );
        } else {
            if (env->getWorkspace()->getTree(input_name) != nullptr) {
                env->getWorkspace()->depthFirstSearch(
                              input_name
                            , depthFirstSearchWorkspaceAll(mapit::msgs::Tree)
                            , depthFirstSearchWorkspaceAll(mapit::msgs::Tree)
                            , [&](std::shared_ptr<mapit::msgs::Entity> obj, const ObjectReference& ref, const mapit::Path &path)
                              {
                                  stamps.push_back( mapit::time::from_msg(obj->stamp()) );
                                  return true;
                              }
                            , depthFirstSearchWorkspaceAll(mapit::msgs::Entity)
                            );
            } else {
                log_error("operator octomap2occupancy_grid: entity name \"" + input_name + "\" given in param \"path-entity-stamps\", is neither a entity nor a tree");
                throw MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
            }
        }
    }

    return stamps;
}

mapit::StatusCode operate_octomap2opccupancy_grid(mapit::OperationEnvironment* env)
{
    /** structure:
     * {
     *  <string>"tf-prefix" : ...,              // default ""
     *  <string>"sensor-frame_id" : ...,
     *  <string>"path-octomap" : ...,
     *  <string>"path-occupancy_grid" : ...,
     *  <string>"path-entity-stamps" : ...,     // the stamps of this entities will be used to transform 3D to 2D (typicly the data that where used to generate the map)
     *  <float>"sensor-distance" : ...,
     *  <float>"slice-hight" : ...              // default 0.1
     * }
     */
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    // -- get configs
    std::string tf_prefix = params["tf-prefix"].toString("").toStdString();
    if ( ! params.contains("sensor-frame_id") ) {
        log_error("operator octomap2occupancy_grid: config does not contain sensor-frame_id");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    std::string frame_id_sensor = params["sensor-frame_id"].toString().toStdString();
    if ( ! params.contains("path-octomap") ) {
        log_error("operator octomap2occupancy_grid: config does not contain path-octomap");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    std::string path_octomap = params["path-octomap"].toString().toStdString();
    if ( ! params.contains("path-occupancy_grid") ) {
        log_error("operator octomap2occupancy_grid: config does not contain path-occupancy_grid");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    std::string path_occupancy_grid = params["path-occupancy_grid"].toString().toStdString();
    if ( ! params.contains("path-entity-stamps") ) {
        log_error("operator octomap2occupancy_grid: config does not contain path-entity-stamps");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    // -- get entity for stamps
    std::vector<mapit::time::Stamp> stamps;
    try {
        stamps = get_param_input(env, params);
    } catch (mapit::StatusCode status) {
        return status;
    }
    float distance_sensor = static_cast<float>( params["sensor-distance"].toDouble( static_cast<double>(std::numeric_limits<float>::max()) ) );
    float slice_hight = static_cast<float>( params["slice-hight"].toDouble(0.1) );

    // -- get workspace and transform buffer
    mapit::operators::WorkspaceWritable* workspace = env->getWorkspace();

    mapit::tf2::BufferCore tf_buffer(workspace, tf_prefix);

    // -- get entity octomap
    std::shared_ptr<mapit::msgs::Entity> octomap_entity = workspace->getEntity(path_octomap);
    if ( ! octomap_entity ) {
        log_error("operator octomap2occupancy_grid: entity \"" << path_octomap << "\" does not exists.");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    if ( 0 != octomap_entity->type().compare(mapit::entitytypes::Octomap::TYPENAME()) ) {
        log_error("operator octomap2occupancy_grid: entity \"" << path_octomap << "\" is of type " << octomap_entity->type()
                                                               << " but " << mapit::entitytypes::Octomap::TYPENAME() << " is required.");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    std::string frame_id_map = octomap_entity->frame_id();
    std::shared_ptr<mapit::entitytypes::Octomap> octomap_entitydata = std::static_pointer_cast<mapit::entitytypes::Octomap>( workspace->getEntitydataReadOnly(path_octomap) );

    std::shared_ptr<octomap::OcTree> octomap = octomap_entitydata->getData();
    double octomap_resolution = octomap->getResolution();
    // -- for all points in the transformed plane
    // TODO does this work, or do I have to use the transformed x, y, z
    double om_m_x, om_m_y, om_m_z;
    float octomap_max_x, octomap_max_y, octomap_max_z;
    octomap->getMetricMax(om_m_x, om_m_y, om_m_z);
    octomap_max_x = static_cast<float>(om_m_x);
    octomap_max_y = static_cast<float>(om_m_y);
    octomap_max_z = static_cast<float>(om_m_z);

    std::shared_ptr<mapit::entitytypes::Grid2DHelper> grid = std::make_shared<mapit::entitytypes::Grid2DHelper>();
    mapit::msgs::Pose origin;
    origin.mutable_translation()->set_x(0);
    origin.mutable_translation()->set_y(0);
    float grid_size_factor = 1.3f;
    grid->initGrid(octomap_max_x * grid_size_factor, octomap_max_y * grid_size_factor, static_cast<float>(octomap_resolution), origin);

    Eigen::Affine3f pose;
    Eigen::Affine2f pose_2d; // for the 3D to 2D mapping, this is only x, y, ori
    // -- for each stamp
    size_t slice_number = 1;
    for (mapit::time::Stamp stamp : stamps) {
        log_info("operator octomap2occupancy_grid: at slice # " << slice_number++ << " / " << stamps.size());
        // -- get pose from transform
        try {
            mapit::tf::TransformStamped tf = tf_buffer.lookupTransform(frame_id_map, frame_id_sensor, stamp);
            pose = tf.transform.translation * tf.transform.rotation;
            Eigen::Quaternionf q_yaw = tf.transform.rotation;
            q_yaw.x() = 0;
            q_yaw.y() = 0;
            q_yaw.normalize();
            Eigen::Vector3f euler = q_yaw.toRotationMatrix().eulerAngles(0, 1, 2);
            Eigen::Rotation2Df rot_2d(euler[2]);
            Eigen::Translation2f trans_2d(  tf.transform.translation.x()
                                          , tf.transform.translation.y());
            pose_2d = trans_2d * rot_2d;
        } catch (...) {
            log_error("operator octomap2occupancy_grid: can't lookup transform " << frame_id_map << " -> " << frame_id_sensor
                                                                                 << " at time " << mapit::time::to_string(stamp));
            return MAPIT_STATUS_ERR_UNKNOWN;
        }

        for (float x = 0; x <= distance_sensor && x < octomap_max_x; x += octomap_resolution) {
            for (float y = 0; y <= distance_sensor && y < octomap_max_y; y += octomap_resolution) {
                for (float z = 0; z <= slice_hight && z < octomap_max_z; z += octomap_resolution) {
                    // check all 8 poses (+/-x, +/-y and +/-z)
                    std::vector<Eigen::Vector3f> vecs;
                    vecs.push_back( Eigen::Vector3f( x,  y,  z) );
                    vecs.push_back( Eigen::Vector3f( x,  y, -z) );
                    vecs.push_back( Eigen::Vector3f( x, -y,  z) );
                    vecs.push_back( Eigen::Vector3f( x, -y, -z) );
                    vecs.push_back( Eigen::Vector3f(-x,  y,  z) );
                    vecs.push_back( Eigen::Vector3f(-x,  y, -z) );
                    vecs.push_back( Eigen::Vector3f(-x, -y,  z) );
                    vecs.push_back( Eigen::Vector3f(-x, -y, -z) );
                    for (Eigen::Vector3f vec : vecs) {
                        // transform point on plane
                        Eigen::Vector3f vec_tf = pose * vec;
                        // get node
                        octomap::OcTreeNode *node = octomap->search(  static_cast<double>(vec_tf.x())
                                                                    , static_cast<double>(vec_tf.y())
                                                                    , static_cast<double>(vec_tf.z())
                                                                    );
                        // get greyvalue 0 = occupied, 128 = unknown, 255 = free
                        int greyvalue_new = mapit::entitytypes::Grid2DHelper::GRID_UNKNOWN;
                        if ( node ) {
                            if ( octomap->isNodeOccupied(node) ) {
                                greyvalue_new = mapit::entitytypes::Grid2DHelper::GRID_OCCUPIED;
                            } else {
                                greyvalue_new = mapit::entitytypes::Grid2DHelper::GRID_FREE;
                            }
                        }

                        // transform point on plane to 2D map from pose(x, y, ori) and vec(x, y)
                        Eigen::Vector2f vec_tf_2d = pose_2d * Eigen::Vector2f(vec.x(), vec.y());
//                        log_info("write at ( " << vec_tf_2d.transpose() << " ) = " << greyvalue);
                        try {
                            int greyvalue = std::max( greyvalue_new, grid->getProbability(vec_tf_2d.x(), vec_tf_2d.y()) );
                            grid->setProbability(vec_tf_2d.x(), vec_tf_2d.y(), greyvalue);
                        } catch (std::out_of_range e) {
                            log_error("operator octomap2occupancy_grid: try to write out of 2D Grid with size factor of " << grid_size_factor);
                            // TODO increase and restart?
//                            grid_size_factor += 0.5f;
//                            jump back to grid->initGrid
                            return MAPIT_STATUS_ERROR;
                        }
                    }
                }
            }
        }

//        // Test export to ansii art on screen
//        float used_width = 20;
//        for (float x = -used_width; x < used_width; x += octomap_resolution ) {
//            std::string line = "";
//            for (float y = -used_width; y < used_width; y += octomap_resolution ) {
//                int prob = grid->getProbability(x, y);
//                if (prob == mapit::entitytypes::Grid2DHelper::GRID_FREE) {
//                    line += " ";
//                } else if (prob == mapit::entitytypes::Grid2DHelper::GRID_OCCUPIED) {
//                    line += "X";
//                } else {
//                    line += "-";
//                }
//                line += " ";
//            }
//            log_info( line );
//        }
//        log_info("");
    }

    std::shared_ptr<mapit::msgs::Entity> grid_entity = std::make_shared<mapit::msgs::Entity>();
    grid_entity->set_type( mapit::entitytypes::Grid2D::TYPENAME() );
    grid_entity->set_frame_id( octomap_entity->frame_id() );
    grid_entity->mutable_stamp()->set_sec( octomap_entity->stamp().sec() );
    grid_entity->mutable_stamp()->set_nsec( octomap_entity->stamp().nsec() );
    workspace->storeEntity(path_occupancy_grid, grid_entity);
    std::shared_ptr<mapit::entitytypes::Grid2D> grid_ed = std::static_pointer_cast<mapit::entitytypes::Grid2D>( workspace->getEntitydataForReadWrite( path_occupancy_grid ) );
    grid_ed->setData( grid );

    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME, "convert an octree to and occupancy grid (3D map -> 2D map)", "fhac", OPERATOR_VERSION, "any", true, &operate_octomap2opccupancy_grid)
