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
filter_pointcloud(mapit::OperationEnvironment* env, const std::string& input_name, const float& min, const float& max)
{
    // get entity
    std::shared_ptr<mapit::msgs::Entity> input_entity = env->getWorkspace()->getEntity(input_name);
    if ( input_entity == nullptr ) {
        log_error("operator filter_pointcloud_passthrough_polar: entity \"" + input_name + "\" dosn't exists");
        throw MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    if ( input_entity->type() != PointcloudEntitydata::TYPENAME()) {
        log_error("operator filter_pointcloud_passthrough_polar: entity \"" + input_name + "\" is of wrong type " + input_entity->type());
        throw MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }

    // get entity data
    std::shared_ptr<mapit::AbstractEntitydata> abstract_input = env->getWorkspace()->getEntitydataForReadWrite( input_name );
    assert(abstract_input);
    std::shared_ptr<PointcloudEntitydata> input_entity_data = std::static_pointer_cast<PointcloudEntitydata>( abstract_input );
    assert(input_entity_data);

    std::shared_ptr<pcl::PCLPointCloud2> input_pc2 = input_entity_data->getData();
    std::shared_ptr<pcl::PCLPointCloud2> out = std::make_shared<pcl::PCLPointCloud2>();
    out->header = input_pc2->header;
    out->fields = input_pc2->fields;
    out->is_bigendian = input_pc2->is_bigendian;
    out->point_step   = input_pc2->point_step;
    out->row_step     = input_pc2->row_step;
    out->is_dense     = input_pc2->is_dense;
    out->data.resize(input_pc2->data.size ());

    // Get X-Y-Z indices
    int x_idx = pcl::getFieldIndex (*input_pc2, "x");
    int y_idx = pcl::getFieldIndex (*input_pc2, "y");
    int z_idx = pcl::getFieldIndex (*input_pc2, "z");

    if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
        log_error("pointcloud2: Input dataset has no X-Y-Z coordinates! Cannot convert to Eigen format.");
        throw MAPIT_STATUS_INVALID_DATA;
    }

    if (input_pc2->fields[x_idx].datatype != pcl::PCLPointField::FLOAT32 ||
        input_pc2->fields[y_idx].datatype != pcl::PCLPointField::FLOAT32 ||
        input_pc2->fields[z_idx].datatype != pcl::PCLPointField::FLOAT32) {
        log_error("pointcloud2: X-Y-Z coordinates not floats. Currently only floats are supported.");
        throw MAPIT_STATUS_INVALID_DATA;
    }

    Eigen::Array4i xyz_offset (input_pc2->fields[x_idx].offset, input_pc2->fields[y_idx].offset, input_pc2->fields[z_idx].offset, 0);
    unsigned int data_offset_in = 0;
    unsigned int data_offset_out = 0;

    double min_square = min * min;
    double max_squate = max * max;
    unsigned int number_fields = 0;
    for (size_t i = 0; i < input_pc2->width * input_pc2->height; ++i) {
        Eigen::Vector4f pt (*(float*)&input_pc2->data[xyz_offset[0]], *(float*)&input_pc2->data[xyz_offset[1]], *(float*)&input_pc2->data[xyz_offset[2]], 1);

        double pt_square = pt.x() * pt.x() + pt.y() * pt.y() + pt.z() * pt.z();
        if (min_square < pt_square && max_squate > pt_square) {
            memcpy(&out->data[data_offset_out], &input_pc2->data[data_offset_in], input_pc2->point_step);
            data_offset_out += out->point_step;
            number_fields++;
        }

        xyz_offset += input_pc2->point_step;
        data_offset_in += input_pc2->point_step;
    }

    out->width = number_fields;
    out->height = 1;

    input_entity_data->setData(out);
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
                log_error("operator filter_pointcloud_passthrough_polar: cfg \"input\" is not an array of strings");
                throw MAPIT_STATUS_INVALID_ARGUMENT;
            }
            input_list.push_back( input.toString().toStdString() );
        }
    } else {
        log_error("operator filter_pointcloud_passthrough_polar: cfg \"input\" does not is a string or array of strings");
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

mapit::StatusCode operate_filter_pcs_passthrough_polar(mapit::OperationEnvironment* env)
{
    /** structure:
     * {
     *  <string>"input"[] : ..., // can either be, a list of entities or a tree containing enteties
     *  <double>"min" : ..., // min distance (everthing below this will be removed)
     *  <double>"max" : ..., // min distance (everthing above this will be removed)
     * }
     */
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    log_info( "operator filter_pointcloud_passthrough_polar: params:" + env->getParameters() );
    QJsonObject params(paramsDoc.object());

    // --cfg - input
    std::vector<std::string> input_names;
    try {
        input_names = get_param_input(env, params);
    } catch (mapit::StatusCode status) {
        return status;
    }

    float min = params["min"].toDouble();
    float max = params["max"].toDouble();

    // add all pointclouds from source
    for (std::string input_name : input_names) {
        try {
            filter_pointcloud(env, input_name, min, max);
        } catch (mapit::StatusCode status) {
            return status;
        }
    }

    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME, "fuse several pointclouds into one", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, true, &operate_filter_pcs_passthrough_polar)
