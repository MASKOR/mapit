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

#include "reg_global_elch.h"

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

mapit::RegGlobalELCH::RegGlobalELCH(mapit::OperationEnvironment* env, mapit::StatusCode &status)
{
    status = MAPIT_STATUS_OK;

    try {
        reg_helper_ = new mapit::RegistrationStorageHelper(env);
    } catch (mapit::StatusCode err) {
        log_error("reg_global_elch: construct RegistrationStorageHelper error code " << err);
        status = err;
        return;
    }

    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    // create and configure ICP
    icp_ = boost::make_shared<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();

    cfg_icp_set_maximum_iterations_ = params.contains("icp-maximum-iterations") && params["icp-maximum-iterations"].toInt() != 0;
    if ( cfg_icp_set_maximum_iterations_ ) {
        cfg_icp_maximum_iterations_ = params["icp-maximum-iterations"].toInt();
        log_info("reg_global_elch: use cfg \"icp-maximum-iterations\": " << cfg_icp_maximum_iterations_);

        icp_->setMaximumIterations( cfg_icp_maximum_iterations_ );
    }
    cfg_icp_set_max_correspondence_distance_ = params.contains("icp-max-correspondence-distance") && params["icp-max-correspondence-distance"].toDouble() != 0;
    if ( cfg_icp_set_max_correspondence_distance_ ) {
        cfg_icp_max_correspondence_distance_ = params["icp-max-correspondence-distance"].toDouble();
        log_info("reg_global_elch: use cfg \"icp-max-correspondence-distance\": " << cfg_icp_max_correspondence_distance_);

        icp_->setMaxCorrespondenceDistance(cfg_icp_max_correspondence_distance_);
    }
//    cfg_icp_set_transformation_epsilon_ = params.contains("icp-transformation-epsilon") && params["icp-transformation-epsilon"].toDouble() != 0;
//    if ( cfg_icp_set_transformation_epsilon_ ) {
//        cfg_icp_transformation_epsilon_ = params["icp-transformation-epsilon"].toDouble();
//        log_info("reg_local_icp: use cfg \"icp-transformation-epsilon\": " << cfg_icp_transformation_epsilon_);
//    }
//    cfg_icp_set_euclidean_fitness_epsilon_ = params.contains("icp-euclidean-fitness-epsilon") && params["icp-euclidean-fitness-epsilon"].toDouble() != 0;
//    if ( cfg_icp_set_euclidean_fitness_epsilon_ ) {
//        cfg_icp_euclidean_fitness_epsilon_ = params["icp-euclidean-fitness-epsilon"].toDouble();
//        log_info("reg_local_icp: use cfg \"icp-euclidean-fitness-epsilon\": " << cfg_icp_euclidean_fitness_epsilon_);
//    }
//    icp_->setRANSACOutlierRejectionThreshold();

    // create and configure ELCH
    elch_ = boost::make_shared<pcl::registration::ELCH<pcl::PointXYZ>>();
    elch_->setReg(icp_);
}

mapit::StatusCode
mapit::RegGlobalELCH::operate()
{
    CloudVector clouds;
    for (std::string cfg_input_one : reg_helper_->cfg_input_) {
        // get input cloud
        mapit::time::Stamp input_stamp;
        pcl::PCLHeader input_header;
        std::shared_ptr<PointcloudEntitydata> entitydata_input;
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_pc;
        try {
            input_pc = reg_helper_->get_pointcloud( cfg_input_one, input_stamp, input_header, entitydata_input );
        } catch (mapit::StatusCode err) {
            log_error("reg_global_elch: Error " << err << " while loading pointcloud");
            return err;
        }
        clouds.push_back( CloudPair(cfg_input_one, input_pc) );
        elch_->addPointCloud( input_pc );
    }

    int first = 0, last = 0;

    for (size_t i = 0; i < clouds.size (); i++) {
        if (loopDetection (int(i), clouds, 10.0, first, last)) {
            log_info("reg_global_elch: Loop between " << first << " (" << clouds[first].first << ") and " << last << " (" << clouds[last].first << ")");
            elch_->setLoopStart( first );
            elch_->setLoopEnd( last );
            elch_->compute();
        }
    }

    for (size_t i = 0; i < clouds.size (); i++) {
        std::string result_filename (clouds[i].first);
        log_warn("reg_global_elch: saving result to " << result_filename << " no other handeling of the result is implemented");
        log_warn("reg_global_elch: only XYZ will survive, intensity and color will be lost");
        std::shared_ptr<pcl::PCLPointCloud2> pc_out = std::make_shared<pcl::PCLPointCloud2>();
        pcl::toPCLPointCloud2(*clouds[i].second, *pc_out);

        mapit::time::Stamp input_stamp;
        pcl::PCLHeader input_header;
        std::shared_ptr<PointcloudEntitydata> entitydata_input;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_pc;
        // TODO: this is probably the most inefficient way to store it ...
        try {
            input_pc = reg_helper_->get_pointcloud( result_filename, input_stamp, input_header, entitydata_input );
        } catch(mapit::StatusCode err) {
            log_error("bla");
            return err;
        }
        pc_out->header = input_header;

        // TODO store data here
        entitydata_input->setData(pc_out);
    }

    return MAPIT_STATUS_OK;
}

bool
mapit::RegGlobalELCH::loopDetection (int end, const CloudVector &clouds, double dist, int &first, int &last)
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
mapit::RegGlobalELCH::elch_execute(  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input
                        , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& target
                        , pcl::PointCloud<pcl::PointXYZ>& result_pc
                        , Eigen::Affine3f& result_transform
                        , double& fitness_score)
{
    return false;
}
