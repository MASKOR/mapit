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

    // create and configure ELCH
    lum_ = boost::make_shared<pcl::registration::LUM<pcl::PointXYZ>>();
}

mapit::StatusCode
mapit::RegGlobalLUM::operate()
{
    // load pointclouds and add to LUM
    log_info("reg_global_lum: add Pointclouds");
    for (std::string cfg_input_one : reg_helper_->cfg_input_) {
        // get input cloud
        mapit::time::Stamp input_stamp;
        pcl::PCLHeader input_header;
        std::shared_ptr<PointcloudEntitydata> entitydata_input;
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_pc;
        try {
            input_pc = reg_helper_->get_pointcloud( cfg_input_one, input_stamp, input_header, entitydata_input );
        } catch (mapit::StatusCode err) {
            log_error("reg_global_lum: Error " << err << " while loading pointcloud");
            return err;
        }
        lum_->addPointCloud( input_pc );
    }

    // add correspondance
    log_info("reg_global_lum: and correspondances and search for loops");
    for (size_t pc_id = 1; pc_id < lum_->getNumVertices(); ++pc_id) {
        // iterative
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_source = lum_->getPointCloud(pc_id - 1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_target = lum_->getPointCloud(pc_id);
        pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
        corr_est.setInputSource( pc_source );
        corr_est.setInputTarget( pc_target );
        pcl::CorrespondencesPtr corrs = boost::make_shared<pcl::Correspondences>();
        corr_est.determineReciprocalCorrespondences( *corrs );
        lum_->setCorrespondences(pc_id - 1, pc_id, corrs);

        // search for loops
        Eigen::Vector4f c_loop, c_target;
        pcl::compute3DCentroid(*pc_target, c_target);
        for (size_t loop_id = 0; loop_id < pc_id - 1; ++loop_id) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr pc_loop = lum_->getPointCloud(loop_id);

            pcl::compute3DCentroid(*pc_loop, c_loop);
            Eigen::Vector4f diff = c_target - c_loop;
            double norm = diff.norm();

            // loop => add correspondance
            if (norm < 7.) {
                log_info("reg_global_lum: loop between " << loop_id << " and " << pc_id);
                pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
                corr_est.setInputSource( pc_loop );
                corr_est.setInputTarget( pc_target );
                pcl::CorrespondencesPtr corrs = boost::make_shared<pcl::Correspondences>();
                corr_est.determineReciprocalCorrespondences( *corrs );
                lum_->setCorrespondences(loop_id, pc_id, corrs);
            }
        }
    }

    log_info("reg_global_lum: start computation");
    // Change the computation parameters
    lum_->setMaxIterations(50);
    lum_->setConvergenceThreshold (0.0);

    lum_->compute ();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out = lum_->getConcatenatedCloud();

    std::string result_filename = "/LUM/out";
    log_warn("reg_global_lum: saving result to " << result_filename << " no other handeling of the result is implemented");
    log_warn("reg_global_lum: only XYZ will survive, intensity and color will be lost");
    std::shared_ptr<pcl::PCLPointCloud2> pc_out = std::make_shared<pcl::PCLPointCloud2>();
    pcl::toPCLPointCloud2(*cloud_out, *pc_out);



    std::shared_ptr<mapit::msgs::Entity> entity = std::make_shared<mapit::msgs::Entity>();
//    entity->set_frame_id("");
//    ...
    entity->set_type( PointcloudEntitydata::TYPENAME() );
    reg_helper_->workspace_->storeEntity(result_filename, entity);

    std::shared_ptr<mapit::AbstractEntitydata> abstract_entitydata = reg_helper_->workspace_->getEntitydataForReadWrite( result_filename );
    if ( 0 != std::strcmp( abstract_entitydata->type(), PointcloudEntitydata::TYPENAME() )) {
        return MAPIT_STATUS_ERR_DB_CORRUPTION;
    }
    std::shared_ptr<PointcloudEntitydata> entitydata = std::static_pointer_cast<PointcloudEntitydata>( abstract_entitydata );

    entitydata->setData(pc_out);

//    for(int i = 0; i < lum_->getNumVertices(); i++) {
//      Eigen::Affine3f tf = lum_->getTransformation(i);
//      log_info("tf for " << i << "\n" << tf.matrix());
//    }

    return MAPIT_STATUS_OK;
}

bool
mapit::RegGlobalLUM::loopDetection(int end, const CloudVector &clouds, double dist, int &first, int &last)
{
  static double min_dist = -1;
  int state = 0;

  for (int i = end-1; i > 0; i--)
  {
    Eigen::Vector4f cstart, cend;
    //TODO use pose of scan
    pcl::compute3DCentroid (*(clouds[i].second), cstart);
    pcl::compute3DCentroid (*(clouds[end].second), cend);
    Eigen::Vector4f diff = cend - cstart;

    double norm = diff.norm ();

    //std::cout << "distance between " << i << " and " << end << " is " << norm << " state is " << state << std::endl;

    if (state == 0 && norm > dist)
    {
      state = 1;
      //std::cout << "state 1" << std::endl;
    }
    if (state > 0 && norm < dist)
    {
      state = 2;
      //std::cout << "loop detected between scan " << i << " (" << clouds[i].first << ") and scan " << end << " (" << clouds[end].first << ")" << std::endl;
      if (min_dist < 0 || norm < min_dist)
      {
        min_dist = norm;
        first = i;
        last = end;
      }
    }
  }
  //std::cout << "min_dist: " << min_dist << " state: " << state << " first: " << first << " end: " << end << std::endl;
  if (min_dist > 0 && (state < 2 || end == int (clouds.size ()) - 1)) //TODO
  {
    min_dist = -1;
    return true;
  }
  return false;
}

bool
mapit::RegGlobalLUM::elch_execute(  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input
                        , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& target
                        , pcl::PointCloud<pcl::PointXYZ>& result_pc
                        , Eigen::Affine3f& result_transform
                        , double& fitness_score)
{
    return false;
}
