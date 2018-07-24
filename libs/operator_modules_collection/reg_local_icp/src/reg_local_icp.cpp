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

#include "reg_local_icp.h"

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

#include <pcl/registration/icp.h>

mapit::RegLocalICP::RegLocalICP(mapit::OperationEnvironment* env, mapit::StatusCode &status)
{
    status = MAPIT_STATUS_OK;

    try {
        reg_helper_ = new mapit::RegistrationStorageHelper(env);
    } catch (int err) {
        log_error("reg_local_icp: construct RegistrationStorageHelper error code " << err);
    }

    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    cfg_use_metascan_ = params.contains("use-metascan") ? params["use-metascan"].toBool() : false;
    if (cfg_use_metascan_) {
        log_info("reg_local_icp: use metascan on target (with voxelgrid TODO, not yet implemented)");
    } else {
        log_info("reg_local_icp: do not use metascan");
    }

    // get algorithm params
    cfg_icp_set_maximum_iterations_ = params.contains("icp-maximum-iterations") && params["icp-maximum-iterations"].toInt() != 0;
    if ( cfg_icp_set_maximum_iterations_ ) {
        cfg_icp_maximum_iterations_ = params["icp-maximum-iterations"].toInt();
        log_info("reg_local_icp: use cfg \"icp-maximum-iterations\": " << cfg_icp_maximum_iterations_);
    }
    cfg_icp_set_max_correspondence_distance_ = params.contains("icp-max-correspondence-distance") && params["icp-max-correspondence-distance"].toDouble() != 0;
    if ( cfg_icp_set_max_correspondence_distance_ ) {
        cfg_icp_max_correspondence_distance_ = params["icp-max-correspondence-distance"].toDouble();
        log_info("reg_local_icp: use cfg \"icp-max-correspondence-distance\": " << cfg_icp_max_correspondence_distance_);
    }
    cfg_icp_set_transformation_epsilon_ = params.contains("icp-transformation-epsilon") && params["icp-transformation-epsilon"].toDouble() != 0;
    if ( cfg_icp_set_transformation_epsilon_ ) {
        cfg_icp_transformation_epsilon_ = params["icp-transformation-epsilon"].toDouble();
        log_info("reg_local_icp: use cfg \"icp-transformation-epsilon\": " << cfg_icp_transformation_epsilon_);
    }
    cfg_icp_set_euclidean_fitness_epsilon_ = params.contains("icp-euclidean-fitness-epsilon") && params["icp-euclidean-fitness-epsilon"].toDouble() != 0;
    if ( cfg_icp_set_euclidean_fitness_epsilon_ ) {
        cfg_icp_euclidean_fitness_epsilon_ = params["icp-euclidean-fitness-epsilon"].toDouble();
        log_info("reg_local_icp: use cfg \"icp-euclidean-fitness-epsilon\": " << cfg_icp_euclidean_fitness_epsilon_);
    }
}

mapit::StatusCode
mapit::RegLocalICP::operate()
{
    try {
        reg_helper_->operate_pairwise( std::bind(&mapit::RegLocalICP::icp_execute, this
                                                 , std::placeholders::_1
                                                 , std::placeholders::_2
                                                 , std::placeholders::_3
                                                 , std::placeholders::_4
                                                 , std::placeholders::_5
                                                 , std::placeholders::_6)
                                       , cfg_use_metascan_);
    } catch(mapit::StatusCode err) {
        return err;
    }

    return MAPIT_STATUS_OK;
}

bool
mapit::RegLocalICP::icp_execute(  const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input
                                , const Eigen::Affine3f& initial_guess_transform
                                , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target
                                , pcl::PointCloud<pcl::PointXYZ>& result_pc
                                , Eigen::Affine3f& result_transform
                                , double& fitness_score)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(input);
    icp.setInputTarget(target);

    if (cfg_icp_set_maximum_iterations_) {
        icp.setMaximumIterations(cfg_icp_maximum_iterations_);
    }
    if (cfg_icp_set_max_correspondence_distance_) {
        icp.setMaxCorrespondenceDistance(cfg_icp_max_correspondence_distance_);
    }
    if (cfg_icp_set_transformation_epsilon_) {
        icp.setTransformationEpsilon(cfg_icp_transformation_epsilon_);
    }
    if (cfg_icp_set_euclidean_fitness_epsilon_) {
        icp.setEuclideanFitnessEpsilon(cfg_icp_euclidean_fitness_epsilon_);
    }

    icp.align(result_pc, initial_guess_transform.matrix());

    bool has_converged = icp.hasConverged();
    result_transform = Eigen::Affine3f( icp.getFinalTransformation() );
    fitness_score = icp.getFitnessScore();

    if (cfg_use_metascan_ && has_converged) {
        *target += result_pc;
    }

    return has_converged;
}
