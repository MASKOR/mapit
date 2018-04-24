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

#include "reg_local.h"

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

mapit::RegLocal::RegLocal(mapit::OperationEnvironment* env, mapit::StatusCode &status)
{
    status = MAPIT_STATUS_OK;

    try {
        reg_helper_ = new mapit::RegistrationStorageHelper(env);
    } catch (int err) {
        log_error("reg_local: construct RegistrationStorageHelper error code " << err);
    }

    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    cfg_use_metascan_ = params.contains("use-metascan") ? params["use-metascan"].toBool() : false;
    if (cfg_use_metascan_) {
        log_info("reg_local: use metascan on target (with voxelgrid TODO, not yet implemented)");
    } else {
        log_info("reg_local: do not use metascan");
    }

    // get algorithm
    std::string cfg_matching_algorithm_str = params.contains("matching-algorithm") ? params["matching-algorithm"].toString().toStdString() : "";
    if ( 0 == cfg_matching_algorithm_str.compare("icp")) {
        cfg_matching_algorithm_ = MatchingAlgorithm::ICP;
        log_info("reg_local: \"matching-algorithm\" is ICP");
        mapit::StatusCode status_icp = get_cfg_icp(params);
        if ( ! mapitIsOk(status_icp) ) {
            status = status_icp;
            return;
        }
    } else {
        log_error("reg_local: \"matching-algorithm\" not specified, going to use ICP");
        cfg_matching_algorithm_ = MatchingAlgorithm::ICP;
        mapit::StatusCode status_icp = get_cfg_icp(params);
        if ( ! mapitIsOk(status_icp) ) {
            status = status_icp;
            return;
        }
    }
}

mapit::StatusCode
mapit::RegLocal::operate()
{
    try {
        reg_helper_->operate_pairwise( std::bind(&mapit::RegLocal::icp_execute, this
                                                 , std::placeholders::_1
                                                 , std::placeholders::_2
                                                 , std::placeholders::_3
                                                 , std::placeholders::_4
                                                 , std::placeholders::_5) );
    } catch(mapit::StatusCode err) {
        return err;
    }

    return MAPIT_STATUS_OK;
}

mapit::StatusCode
mapit::RegLocal::get_cfg_icp(const QJsonObject &params)
{
    cfg_icp_set_maximum_iterations_ = params.contains("icp-maximum-iterations") && params["icp-maximum-iterations"].toInt() != 0;
    if ( cfg_icp_set_maximum_iterations_ ) {
        cfg_icp_maximum_iterations_ = params["icp-maximum-iterations"].toInt();
        log_info("reg_local/ICP: use cfg \"icp-maximum-iterations\": " << cfg_icp_maximum_iterations_);
    }
    cfg_icp_set_max_correspondence_distance_ = params.contains("icp-max-correspondence-distance") && params["icp-max-correspondence-distance"].toDouble() != 0;
    if ( cfg_icp_set_max_correspondence_distance_ ) {
        cfg_icp_max_correspondence_distance_ = params["icp-max-correspondence-distance"].toDouble();
        log_info("reg_local/ICP: use cfg \"icp-max-correspondence-distance\": " << cfg_icp_max_correspondence_distance_);
    }
    cfg_icp_set_transformation_epsilon_ = params.contains("icp-transformation-epsilon") && params["icp-transformation-epsilon"].toDouble() != 0;
    if ( cfg_icp_set_transformation_epsilon_ ) {
        cfg_icp_transformation_epsilon_ = params["icp-transformation-epsilon"].toDouble();
        log_info("reg_local/ICP: use cfg \"icp-transformation-epsilon\": " << cfg_icp_transformation_epsilon_);
    }
    cfg_icp_set_euclidean_fitness_epsilon_ = params.contains("icp-euclidean-fitness-epsilon") && params["icp-euclidean-fitness-epsilon"].toDouble() != 0;
    if ( cfg_icp_set_euclidean_fitness_epsilon_ ) {
        cfg_icp_euclidean_fitness_epsilon_ = params["icp-euclidean-fitness-epsilon"].toDouble();
        log_info("reg_local/ICP: use cfg \"icp-euclidean-fitness-epsilon\": " << cfg_icp_euclidean_fitness_epsilon_);
    }

    return MAPIT_STATUS_OK;
}

bool
mapit::RegLocal::icp_execute(  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input
                        , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& target
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

    icp.align(result_pc);

    bool has_converged = icp.hasConverged();
    result_transform = Eigen::Affine3f( icp.getFinalTransformation() );
    fitness_score = icp.getFitnessScore();

    if (cfg_use_metascan_ && has_converged) {
        *target += result_pc;
    }

    return has_converged;
}
