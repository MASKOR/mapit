/*******************************************************************************
 *
 * Copyright 2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

// since this code is heavely based on "pcl/tools/elch.cpp" we also add there licence header

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2011, Willow Garage, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 * $Id$
 *
 */

#include "reg_global_lum.h"

#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>
#include <mapit/errorcodes.h>
#include <mapit/depthfirstsearch.h>
#include <mapit/logging.h>

#include <mapit/layertypes/tflayer/tf2/buffer_core.h>
#include <mapit/layertypes/pointcloudlayer.h>
#include <pcl/conversions.h>

#include <pcl/registration/correspondence_estimation.h>

#include <mutex>

mapit::RegGlobalLUM::RegGlobalLUM(mapit::OperationEnvironment* env, mapit::StatusCode &status)
{
    status = MAPIT_STATUS_OK;

    try {
        reg_helper_ = new mapit::RegistrationStorageHelper(env);
    } catch (mapit::StatusCode err) {
        log_error("reg_global_lum: construct RegistrationStorageHelper error code " << err);
        status = err;
        return;
    }

    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    // get LUM configs
    cfg_lum_maximum_iterations_ = params["lum-maximum-iterations"].toInt(5);
    log_info("reg_global_lum: cfg \"lum-maximum-iterations\": " << cfg_lum_maximum_iterations_);

    cfg_lum_convergence_threshold_ = static_cast<float>(params["lum-convergence-threshold"].toDouble(0.0));
    log_info("reg_global_lum: cfg \"lum-convergence-threshold\": " << cfg_lum_convergence_threshold_);

    cfg_lum_graph_centroid_distance_ = static_cast<float>(params["lum-graph-centroid-distance"].toDouble(20.0));
    log_info("reg_global_lum: cfg \"lum-graph-centroid-distance\": " << cfg_lum_graph_centroid_distance_);

    // create and configure LUM
    lum_ = boost::make_shared<pcl::registration::LUM<pcl::PointXYZ>>();
}

mapit::StatusCode
mapit::RegGlobalLUM::operate()
{
    try {
        reg_helper_->operate_global(  std::bind(&mapit::RegGlobalLUM::callback_add_pointcloud, this
                                                , std::placeholders::_1)
                                    , std::bind(&mapit::RegGlobalLUM::callback_search_and_process_loops, this)
                                    , std::bind(&mapit::RegGlobalLUM::callback_execute_algorithm, this
                                                , std::placeholders::_1
                                                , std::placeholders::_2
                                                , std::placeholders::_3
                                                , std::placeholders::_4)
                                   );
    } catch(mapit::StatusCode err) {
        return err;
    }

    return MAPIT_STATUS_OK;
}

void
mapit::RegGlobalLUM::callback_add_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pc)
{
    lum_->addPointCloud( input_pc );
}

void
mapit::RegGlobalLUM::callback_search_and_process_loops()
{
    log_info("reg_global_lum: and correspondances and search for loops");
    std::vector<Eigen::Vector4f> centroid;

    centroid.resize( lum_->getNumVertices() );
    #pragma omp parallel for
    for (size_t pc_id = 0; pc_id < lum_->getNumVertices(); ++pc_id) {
        Eigen::Vector4f c;
        pcl::compute3DCentroid(*lum_->getPointCloud(pc_id), c);
        centroid[pc_id] = c;
    }

    std::mutex mtx_logging;
    #pragma omp parallel for
    for (size_t pc_id = lum_->getNumVertices() - 1; pc_id > 0; --pc_id) {
//        mtx_logging.lock();
//        log_info("reg_global_lum: add correspondance for cloud " << pc_id - 1 << " -> " << pc_id);
//        mtx_logging.unlock();
//        // iterative
//        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_source = lum_->getPointCloud(pc_id - 1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_target = lum_->getPointCloud(pc_id);
//        pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
//        corr_est.setInputSource( pc_source );
//        corr_est.setInputTarget( pc_target );
//        pcl::CorrespondencesPtr corrs = boost::make_shared<pcl::Correspondences>();
//        corr_est.determineReciprocalCorrespondences( *corrs );
//        lum_->setCorrespondences(pc_id - 1, pc_id, corrs);

        // search for loops
        Eigen::Vector4f c_loop, c_target;
        c_target = centroid.at(pc_id);
        for (size_t loop_id = 0; loop_id < pc_id; ++loop_id) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr pc_loop = lum_->getPointCloud(loop_id);

            c_loop = centroid.at(loop_id);
            Eigen::Vector4f diff = c_target - c_loop;
            float norm = diff.norm();

            // loop => add correspondance
            if (norm < cfg_lum_graph_centroid_distance_) {
                mtx_logging.lock();
                log_info("reg_global_lum: add correspondance for cloud " << loop_id << " -> " << pc_id);
                mtx_logging.unlock();
                pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
                corr_est.setInputSource( pc_loop );
                corr_est.setInputTarget( pc_target );
                pcl::CorrespondencesPtr corrs = boost::make_shared<pcl::Correspondences>();
                corr_est.determineReciprocalCorrespondences( *corrs );
                lum_->setCorrespondences(loop_id, pc_id, corrs);
            }
        }
    }
}

void
mapit::RegGlobalLUM::callback_execute_algorithm(  mapit::RegistrationStorageHelper::HandleResult& handle_result
                                                , std::vector<Eigen::Affine3f>& out_tfs
                                                , std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& out_pointclouds
                                                , pcl::PointCloud<pcl::PointXYZ>::Ptr out_pointcloud)
{
    log_info("reg_global_lum: start computation");
    // Change the computation parameters
    lum_->setMaxIterations( cfg_lum_maximum_iterations_ );
    lum_->setConvergenceThreshold ( cfg_lum_convergence_threshold_ );

    lum_->compute();

    log_info("reg_global_lum: computation done");

    switch (handle_result) {
//    TODO add this option
//    case mapit::RegistrationStorageHelper::HandleResult::data_new...:
//        out_pointcloud = lum_->getConcatenatedCloud();
//        break;
    case mapit::RegistrationStorageHelper::HandleResult::data_change:
        for(int i = 0; i < lum_->getNumVertices(); ++i) {
            Eigen::Affine3f tf = lum_->getTransformation(i);
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud = lum_->getPointCloud(i);

            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_transformed = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::transformPointCloud(*pointcloud, *pointcloud_transformed, tf);

            out_pointclouds.push_back( pointcloud_transformed );
        }
        break;
    case mapit::RegistrationStorageHelper::HandleResult::tf_add:
    case mapit::RegistrationStorageHelper::HandleResult::tf_combine:
        for(int i = 0; i < lum_->getNumVertices(); ++i) {
            out_tfs.push_back( lum_->getTransformation(i) );
        }
        break;
    default:
        log_error("reg_global_lum: can't handle result " << (size_t)handle_result);
        throw MAPIT_STATUS_ERROR;
        break;
    }
}
