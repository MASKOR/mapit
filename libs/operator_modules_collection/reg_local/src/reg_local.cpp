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
    // get target cloud
    mapit::time::Stamp target_stamp;
    pcl::PCLHeader target_header;
    std::shared_ptr<PointcloudEntitydata> entitydata_target;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target_pc;
    try {
        target_pc = reg_helper_->get_pointcloud( reg_helper_->cfg_target_, target_stamp, target_header, entitydata_target );
    } catch (mapit::StatusCode err) {
        return err;
    }

    // in case of tf-combine, a list is needed
    std::list<std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>>> tf_combine_list;

    for (std::string cfg_input_one : reg_helper_->cfg_input_) {
        // get input cloud
        mapit::time::Stamp input_stamp;
        pcl::PCLHeader input_header;
        std::shared_ptr<PointcloudEntitydata> entitydata_input;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_pc;
        try {
            input_pc = reg_helper_->get_pointcloud( cfg_input_one, input_stamp, input_header, entitydata_input );
        } catch (mapit::StatusCode err) {
            return err;
        }

        // execute algorithm
        double fitness_score = std::numeric_limits<double>::max();
        bool has_converged = false;
        Eigen::Affine3f result_transform;
        pcl::PointCloud<pcl::PointXYZ> result_pc;
        switch (cfg_matching_algorithm_) {
            case MatchingAlgorithm::ICP: {
                icp_execute(input_pc, target_pc, result_pc, result_transform, has_converged, fitness_score);
                break;
            }
            default: {
                log_error("reg_local: can't select algorithm for matching");
                return MAPIT_STATUS_ERROR;
            }
        }

        if ( ! has_converged ) {
          log_error("reg_local: algorithm didn't converged");
          return MAPIT_STATUS_ERROR;
        } else {
            log_info("reg_local: ICP for cloud \"" + cfg_input_one + "\" to \"" + reg_helper_->cfg_target_
                   + "\" finished with fitness score " + std::to_string( fitness_score ));

            if (cfg_use_metascan_) {
                *target_pc += result_pc;
            }
        }

        // handle the result
        switch (reg_helper_->cfg_handle_result_) {
            case RegistrationStorageHelper::HandleResult::data_change: {
                log_info("reg_local: change pointcloud " + cfg_input_one/* + " with tf:"*/);
//                std::cout << transform.matrix() << std::endl;
                log_warn("reg_local: only XYZ will survive, intensity and color will be lost");
                // TODO find way to transform pointcloud2 so that all data survive
                std::shared_ptr<pcl::PCLPointCloud2> icp_out2 = std::make_shared<pcl::PCLPointCloud2>();
                pcl::toPCLPointCloud2(result_pc, *icp_out2);
                icp_out2->header = input_header;
                entitydata_input->setData(icp_out2);
                break;
            }
            case RegistrationStorageHelper::HandleResult::tf_add: {
                try {
                    reg_helper_->mapit_add_tf(input_stamp, result_transform);
                } catch (mapit::StatusCode err) {
                    return err;
                }
                break;
            }
            case RegistrationStorageHelper::HandleResult::tf_combine: {
                // get old tf from buffer
                Eigen::Affine3f tf_in_buffer = Eigen::Affine3f::Identity();
                mapit::time::Stamp stamp = input_stamp;
                unsigned long sec, nsec;
                mapit::time::to_sec_and_nsec(stamp, sec, nsec);
                try {
                    tf::TransformStamped tf = reg_helper_->tf_buffer_->lookupTransform(reg_helper_->cfg_tf_frame_id_, reg_helper_->cfg_tf_child_frame_id_, stamp);
                    tf_in_buffer.translation() << tf.transform.translation.x(), tf.transform.translation.y(), tf.transform.translation.z();
                    tf_in_buffer.rotate( tf.transform.rotation );
                } catch (...) {
                    log_warn("reg_local: tf \""
                           + reg_helper_->cfg_tf_frame_id_ + "\" to \"" + reg_helper_->cfg_tf_child_frame_id_
                           + "\" at time " + std::to_string(sec) + "." + std::to_string(nsec) + " does not exists. Identity will be used");
                    tf_in_buffer = Eigen::Affine3f::Identity();
                }
                // combine tf with tf currently in buffer
                std::shared_ptr<Eigen::Affine3f> tf_combined = std::shared_ptr<Eigen::Affine3f>(new Eigen::Affine3f(tf_in_buffer * result_transform)); // std::make_shared is not possible because its call by value and that does not work with eigen

                // store tf to list
                std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>> tf_pair(stamp, tf_combined);
                tf_combine_list.push_back(tf_pair);
                break;
            }
            default: {
                log_error("reg_local: do not handle result of ICP (no effect), its not yet implemented.");
            }
        }
    }

    // change tfs in mapit
    if ( reg_helper_->cfg_handle_result_ == RegistrationStorageHelper::HandleResult::tf_combine) {
        // get time of input clouds
        mapit::time::Stamp earliest = tf_combine_list.front().first;
        mapit::time::Stamp latest = tf_combine_list.front().first;
        mapit::time::nanoseconds small_time = mapit::time::nanoseconds(1); // smallest time we can have
        for (std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>> tf_time : tf_combine_list) {
            if (earliest > tf_time.first) {
                earliest = tf_time.first;
            }
            if (latest < tf_time.first) {
                latest = tf_time.first;
            }
        }
        // get tf just before we change anything, and add that to the list to store, therefore we do not interpolate between old and new tfs
        try {
            tf::TransformStamped tf = reg_helper_->tf_buffer_->lookupTransform(reg_helper_->cfg_tf_frame_id_, reg_helper_->cfg_tf_child_frame_id_, earliest - small_time);
            Eigen::Affine3f tf_in_buffer = Eigen::Affine3f::Identity();
            tf_in_buffer.translation() << tf.transform.translation.x(), tf.transform.translation.y(), tf.transform.translation.z();
            tf_in_buffer.rotate( tf.transform.rotation );
            // store tf to list
            std::shared_ptr<Eigen::Affine3f> tf_in_buffer_ptr = std::shared_ptr<Eigen::Affine3f>(new Eigen::Affine3f(tf_in_buffer)); // std::make_shared is not possible because its call by value and that does not work with eigen
            std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>> tf_pair(earliest - small_time, tf_in_buffer_ptr);
            tf_combine_list.push_back(tf_pair);
        } catch(...) {
            // when it doesn't work, there is also nothing todo here, so no worries
        }

        // get tf just after we change anything, and add that to the list to store, therefore we do not interpolate between old and new tfs
        try {
            tf::TransformStamped tf = reg_helper_->tf_buffer_->lookupTransform(reg_helper_->cfg_tf_frame_id_, reg_helper_->cfg_tf_child_frame_id_, latest + small_time);
            Eigen::Affine3f tf_in_buffer = Eigen::Affine3f::Identity();
            tf_in_buffer.translation() << tf.transform.translation.x(), tf.transform.translation.y(), tf.transform.translation.z();
            tf_in_buffer.rotate( tf.transform.rotation );
            // store tf to list
            std::shared_ptr<Eigen::Affine3f> tf_in_buffer_ptr = std::shared_ptr<Eigen::Affine3f>(new Eigen::Affine3f(tf_in_buffer)); // std::make_shared is not possible because its call by value and that does not work with eigen
            std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>> tf_pair(latest + small_time, tf_in_buffer_ptr);
            tf_combine_list.push_back(tf_pair);
        } catch(...) {
            // when it doesn't work, there is also nothing todo here, so no worries
        }

        // delete tfs between time of clouds
        try {
            reg_helper_->mapit_remove_tfs(earliest, latest);
        } catch (mapit::StatusCode err) {
            return err;
        }

        // add new tfs
        for (std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>> tf_to_add : tf_combine_list) {
            reg_helper_->mapit_add_tf(tf_to_add.first, *tf_to_add.second);
        }
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

void
mapit::RegLocal::icp_execute(  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input
                        , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target
                        , pcl::PointCloud<pcl::PointXYZ>& result_pc
                        , Eigen::Affine3f& result_transform
                        , bool& has_converged
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

    has_converged = icp.hasConverged();
    result_transform = Eigen::Affine3f( icp.getFinalTransformation() );
    fitness_score = icp.getFitnessScore();
}
