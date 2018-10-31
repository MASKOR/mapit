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
#include <mapit/layertypes/tflayer.h>
#include <mapit/layertypes/tflayer/tf2/buffer_core.h>
#include <mapit/layertypes/tflayer/utils.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <memory>
#include <mapit/errorcodes.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/depthfirstsearch.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

std::shared_ptr<mapit::tf2::BufferCore> buffer_core_;

void
transformPointCloud(const Eigen::Matrix4f &transform, const pcl::PCLPointCloud2 &in,
                    pcl::PCLPointCloud2 &out)
{
    // Get X-Y-Z indices
    int x_idx = pcl::getFieldIndex (in, "x");
    int y_idx = pcl::getFieldIndex (in, "y");
    int z_idx = pcl::getFieldIndex (in, "z");

    if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
        log_error("pointcloud2: Input dataset has no X-Y-Z coordinates! Cannot convert to Eigen format.");
        throw MAPIT_STATUS_INVALID_DATA;
    }

    if (in.fields[x_idx].datatype != pcl::PCLPointField::FLOAT32 ||
        in.fields[y_idx].datatype != pcl::PCLPointField::FLOAT32 ||
        in.fields[z_idx].datatype != pcl::PCLPointField::FLOAT32) {
        log_error("pointcloud2: X-Y-Z coordinates not floats. Currently only floats are supported.");
        throw MAPIT_STATUS_INVALID_DATA;
    }

    // Check if distance is available
    int dist_idx = pcl::getFieldIndex(in, "distance");

    // Copy the other data
    if (&in != &out) {
        out.header = in.header;
        out.height = in.height;
        out.width  = in.width;
        out.fields = in.fields;
        out.is_bigendian = in.is_bigendian;
        out.point_step   = in.point_step;
        out.row_step     = in.row_step;
        out.is_dense     = in.is_dense;
        out.data.resize (in.data.size ());
        // Copy everything as it's faster than copying individual elements
        memcpy(&out.data[0], &in.data[0], in.data.size ());
    }

    Eigen::Array4i xyz_offset (in.fields[x_idx].offset, in.fields[y_idx].offset, in.fields[z_idx].offset, 0);

    for (size_t i = 0; i < in.width * in.height; ++i) {
        Eigen::Vector4f pt (*(float*)&in.data[xyz_offset[0]], *(float*)&in.data[xyz_offset[1]], *(float*)&in.data[xyz_offset[2]], 1);
        Eigen::Vector4f pt_out;

        bool max_range_point = false;
        int distance_ptr_offset = i*in.point_step + in.fields[dist_idx].offset;
        float* distance_ptr = (dist_idx < 0 ? NULL : (float*)(&in.data[distance_ptr_offset]));
        if (!std::isfinite (pt[0]) || !std::isfinite (pt[1]) || !std::isfinite (pt[2])) {
            if (distance_ptr==NULL || !std::isfinite(*distance_ptr)) { // Invalid point
                pt_out = pt;
            }
            else { // max range point
                pt[0] = *distance_ptr;  // Replace x with the x value saved in distance
                pt_out = transform * pt;
                max_range_point = true;
                //std::cout << pt[0]<<","<<pt[1]<<","<<pt[2]<<" => "<<pt_out[0]<<","<<pt_out[1]<<","<<pt_out[2]<<"\n";
            }
        }
        else {
            pt_out = transform * pt;
        }

        if (max_range_point) {
            // Save x value in distance again
            *(float*)(&out.data[distance_ptr_offset]) = pt_out[0];
            pt_out[0] = std::numeric_limits<float>::quiet_NaN();
        }

        memcpy (&out.data[xyz_offset[0]], &pt_out[0], sizeof (float));
        memcpy (&out.data[xyz_offset[1]], &pt_out[1], sizeof (float));
        memcpy (&out.data[xyz_offset[2]], &pt_out[2], sizeof (float));

        xyz_offset += in.point_step;
    }

    // Check if the viewpoint information is present
    int vp_idx = pcl::getFieldIndex (in, "vp_x");
    if (vp_idx != -1) {
        // Transform the viewpoint info too
        for (size_t i = 0; i < out.width * out.height; ++i)
        {
            float *pstep = (float*)&out.data[i * out.point_step + out.fields[vp_idx].offset];
            // Assume vp_x, vp_y, vp_z are consecutive
            Eigen::Vector4f vp_in (pstep[0], pstep[1], pstep[2], 1);
            Eigen::Vector4f vp_out = transform * vp_in;

            pstep[0] = vp_out[0];
            pstep[1] = vp_out[1];
            pstep[2] = vp_out[2];
        }
    }
}

void
transformPointCloud(const mapit::tf::TransformStamped& transform, const pcl::PCLPointCloud2& in,
                    pcl::PCLPointCloud2& out)
{
    Eigen::Affine3f mat;
    mat.matrix().block<3, 3>(0, 0) = transform.transform.rotation.toRotationMatrix();
    mat.matrix()(0, 3) = transform.transform.translation.x();
    mat.matrix()(1, 3) = transform.transform.translation.y();
    mat.matrix()(2, 3) = transform.transform.translation.z();
    transformPointCloud(mat.matrix(), in, out);
}

void
executeTransform(mapit::OperationEnvironment* env, std::shared_ptr<mapit::msgs::Entity> entity, const std::string& target, const std::string& frame_id)
{
    std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydata = env->getWorkspace()->getEntitydataForReadWrite( target );
    std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
    if(entityData == nullptr)
    {
        log_error("transform: Entity Type not yet supported");
        throw MAPIT_STATUS_ERR_NOT_YET_IMPLEMENTED;
    }
    mapit::entitytypes::Pointcloud2Ptr pc2 = entityData->getData();

    // get TF
    mapit::tf::TransformStamped tf = buffer_core_->lookupTransform(frame_id, entity->frame_id(), mapit::time::from_msg(entity->stamp()));

    transformPointCloud(tf, *pc2, *pc2);

    entityData->setData(pc2);
    entity->set_frame_id(frame_id);
    env->getWorkspace()->storeEntity(target, entity);
}

mapit::StatusCode operate_transform(mapit::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    log_info( "transform: params:" + env->getParameters() );
    QJsonObject params(paramsDoc.object());
    std::string frame_id = params["frame_id"].toString().toStdString();

    if ( frame_id.empty() ) {
        log_error("transform: no frame_id given");
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }

    std::string target = params["target"].toString().toStdString();

    buffer_core_ = std::make_shared<mapit::tf2::BufferCore>(env->getWorkspace(), "");

    std::shared_ptr<mapit::msgs::Entity> entity = env->getWorkspace()->getEntity(target);
    if ( entity != nullptr ) {
        // execute on entity
        try {
            executeTransform(env, entity, target, frame_id);
        } catch (mapit::StatusCode err) {
            return err;
        }
        return MAPIT_STATUS_OK;
    } else if ( env->getWorkspace()->getTree(target) ) {
        // execute on tree
        mapit::StatusCode status = MAPIT_STATUS_OK;
        env->getWorkspace()->depthFirstSearch(
                      target
                    , depthFirstSearchWorkspaceAll(mapit::msgs::Tree)
                    , depthFirstSearchWorkspaceAll(mapit::msgs::Tree)
                    , [&](std::shared_ptr<mapit::msgs::Entity> obj, const ObjectReference& ref, const mapit::Path &path)
                        {
                            try {
                                executeTransform(env, obj, path, frame_id);
                            } catch (mapit::StatusCode err) {
                                return false;
                            }

                            return true;
                        }
                    , depthFirstSearchWorkspaceAll(mapit::msgs::Entity)
                    );
        return status;
    } else {
        log_error("transform: target is neither a tree nor entity");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
}

MAPIT_MODULE(OPERATOR_NAME, "transform enteties", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, true, &operate_transform)
