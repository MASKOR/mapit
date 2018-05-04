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

#include <mapit/registration_storage_helper/registration_storage_helper.h>

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
#include <pcl/common/transforms.h>
#include <boost/make_shared.hpp>

mapit::RegistrationStorageHelper::RegistrationStorageHelper(mapit::OperationEnvironment* env)
{
    // pointcloud parameter
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());
    std::list<std::string> input_list;
    if        ( params["input"].isString() ) {
        input_list.push_back( params["input"].toString().toStdString() );
    } else if ( params["input"].isArray() ) {
        for (QJsonValue input : params["input"].toArray() ) {
            if ( ! input.isString() ) {
                log_error("RegistrationStorageHelper: cfg \"input\" does not is a string or array of strings");
                throw MAPIT_STATUS_INVALID_ARGUMENT;
            }
            input_list.push_back( input.toString().toStdString() );
        }
    } else {
        log_error("RegistrationStorageHelper: cfg \"input\" does not is a string or array of strings");
        throw MAPIT_STATUS_INVALID_ARGUMENT;
    }

    // search for all in input
    // if entity -> add to list
    // if tree -> depth search, add all entities to list
    // otherwise error
    for (std::string input_name : input_list) {
        if (env->getWorkspace()->getEntity(input_name) != nullptr) {
            cfg_input_.push_back(input_name);
        } else {
            if (env->getWorkspace()->getTree(input_name) != nullptr) {
                env->getWorkspace()->depthFirstSearch(
                              input_name
                            , depthFirstSearchWorkspaceAll(mapit::msgs::Tree)
                            , depthFirstSearchWorkspaceAll(mapit::msgs::Tree)
                            , [&](std::shared_ptr<mapit::msgs::Entity> obj, const ObjectReference& ref, const mapit::Path &path)
                              {
                                  cfg_input_.push_back(path);
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
    cfg_target_ = params["target"].toString().toStdString();
    if ( ! cfg_target_.empty() ) {
        cfg_input_.remove( cfg_target_ ); // delete the target from the input (in the case it is specified in both)
    }

    std::string output_pointcloud = "RegistrationStorageHelper: executing algorithm on pointclouds [ ";
    for (std::string cfg_input_one : cfg_input_) {
        output_pointcloud += "\"" + cfg_input_one + "\", ";
    }
    output_pointcloud += "]";
    if ( ! cfg_target_.empty() ) {
        output_pointcloud += " to pointcloud \"" + cfg_target_ + "\"";
    }
    log_info(output_pointcloud);

    // config parameter
    cfg_use_frame_id_ = ! (params["frame_id"].isNull() || params["frame_id"].toString().isEmpty());
    if (cfg_use_frame_id_) {
        cfg_frame_id_ = params["frame_id"].toString().toStdString();
        log_info("RegistrationStorageHelper: transform to \"" + cfg_frame_id_ + "\" before executing algorithm");
    } else {
        log_info("RegistrationStorageHelper: do not transform in any frame before executing algorithm");
    }

    // handle parameter
    std::string cfg_handle_result_str = params["handle-result"].toString().toStdString();
    if        ( 0 == cfg_handle_result_str.compare("tf-add") ) {
        cfg_handle_result_ = HandleResult::tf_add;
    } else if ( 0 == cfg_handle_result_str.compare("tf-combine") ) {
        cfg_handle_result_ = HandleResult::tf_combine;
    } else if ( 0 == cfg_handle_result_str.compare("data-change") ) {
        cfg_handle_result_ = HandleResult::data_change;
    } else {
        log_error("RegistrationStorageHelper: can't handle parameter \"handle-result\" = \"" + cfg_handle_result_str + "\"");
        throw MAPIT_STATUS_INVALID_ARGUMENT;
    }

    if (cfg_handle_result_ == HandleResult::tf_add
     || cfg_handle_result_ == HandleResult::tf_combine) {
        cfg_tf_frame_id_ = params["tf-frame_id"].toString().toStdString();
        cfg_tf_child_frame_id_ = params["tf-child_frame_id"].toString().toStdString();
        cfg_tf_is_static_ = params.contains("tf-is_static") ? params["tf-is_static"].toBool() : false;
    } else {
        cfg_tf_frame_id_ = "not-set";
        cfg_tf_child_frame_id_ = "not-set";
        cfg_tf_is_static_ = false;
    }

    // sanity check
    if ( cfg_tf_is_static_ && cfg_input_.size() != 1) {
        log_error("RegistrationStorageHelper: \"tf-is_static\" := true is only allowed for one \"input\" cloud specified");
        throw MAPIT_STATUS_INVALID_ARGUMENT;
    }

    // get env
    workspace_ = env->getWorkspace();
    cfg_tf_prefix_ = params.contains("tf-prefix") ? params["tf-prefix"].toString().toStdString() : "";

    tf_buffer_ = std::make_shared<mapit::tf2::BufferCore>(workspace_, cfg_tf_prefix_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
mapit::RegistrationStorageHelper::get_pointcloud(std::string path, mapit::time::Stamp& stamp, pcl::PCLHeader& header, std::shared_ptr<PointcloudEntitydata>& entitydata)
{
    std::shared_ptr<mapit::msgs::Entity> entity = workspace_->getEntity( path );
    if (entity == nullptr) {
        throw MAPIT_STATUS_INVALID_ARGUMENT;
    }
    std::string frame_id = entity->frame_id();
    stamp = mapit::time::from_msg(entity->stamp());
    std::shared_ptr<mapit::AbstractEntitydata> abstract_entitydata = workspace_->getEntitydataForReadWrite( path );
    if ( 0 != std::strcmp( abstract_entitydata->type(), PointcloudEntitydata::TYPENAME() )) {
        throw MAPIT_STATUS_INVALID_ARGUMENT;
    }
    entitydata = std::static_pointer_cast<PointcloudEntitydata>( abstract_entitydata );
    std::shared_ptr<pcl::PCLPointCloud2> pc2 = entitydata->getData();
    header = pc2->header;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromPCLPointCloud2(*pc2, *pc);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_transformed = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (cfg_use_frame_id_) {
        mapit::tf::TransformStamped tf = tf_buffer_->lookupTransform(cfg_frame_id_, frame_id, stamp);
        pcl::transformPointCloud(*pc, *pc_transformed, tf.transform.translation.vector(), tf.transform.rotation);
    } else {
        *pc_transformed = *pc;
    }

    return pc_transformed;
}

void
mapit::RegistrationStorageHelper::mapit_add_tf(const mapit::time::Stamp& input_stamp, const Eigen::Affine3f& transform)
{
    log_info("RegistrationStorageHelper: add "
           + (cfg_tf_is_static_ ? "static" : "dynamic")
           + " transform from \"" + cfg_tf_frame_id_ + "\" to \"" + cfg_tf_child_frame_id_
           + "\" at time " + mapit::time::to_string(input_stamp) );
//    std::cout << transform.matrix() << std::endl;
    // get infos
    mapit::time::Stamp stamp = input_stamp;
    std::shared_ptr<mapit::msgs::Entity> entity;
    std::shared_ptr<TfEntitydata> ed_tf;
    std::shared_ptr<tf::store::TransformStampedList> ed_d;
    if ( ! mapitIsOk( mapit::tf::store::getOrCreateTransformStampedList(workspace_, cfg_tf_frame_id_, cfg_tf_child_frame_id_, cfg_tf_prefix_, entity, ed_tf, ed_d, cfg_tf_is_static_) )) {
        throw MAPIT_STATUS_ERROR;
    }

    std::unique_ptr<mapit::tf::TransformStamped> tfs = std::make_unique<mapit::tf::TransformStamped>();

    // add data
    tfs->frame_id = cfg_tf_frame_id_;
    tfs->child_frame_id = cfg_tf_child_frame_id_;
    tfs->stamp = stamp;
    tfs->transform.translation.translation() = transform.translation();
    tfs->transform.rotation = transform.rotation();
    ed_d->add_TransformStamped( std::move(tfs), cfg_tf_is_static_);

    unsigned long sec, nsec;
    mapit::time::to_sec_and_nsec(stamp, sec, nsec);
    entity->set_frame_id( cfg_tf_frame_id_ );
    entity->mutable_stamp()->set_sec( sec );
    entity->mutable_stamp()->set_nsec( nsec );

    // write data
    std::string entity_name = cfg_tf_prefix_ + "/" + mapit::tf::store::TransformStampedList::get_entity_name(cfg_tf_frame_id_, cfg_tf_child_frame_id_);
    workspace_->storeEntity(entity_name, entity);
    ed_tf->setData(ed_d);
}

void
mapit::RegistrationStorageHelper::mapit_remove_tfs(const time::Stamp &stamp_start, const time::Stamp &stamp_end)
{
    log_info("RegistrationStorageHelper: remove "
           + " transforms from \"" + cfg_tf_frame_id_ + "\" to \"" + cfg_tf_child_frame_id_
           + "\" between time " + mapit::time::to_string(stamp_start)
           + "\" and " + mapit::time::to_string(stamp_end)
            );
    // get entity and entitydata
    std::string entity_name = cfg_tf_prefix_ + "/" + mapit::tf::store::TransformStampedList::get_entity_name(cfg_tf_frame_id_, cfg_tf_child_frame_id_);
    // get entity and data
    std::shared_ptr<mapit::msgs::Entity> entity = workspace_->getEntity( entity_name );
    if (entity == nullptr) {
        log_warn("RegistrationStorageHelper: transform entity \"" + entity_name + "\" does not exists, do not remove anything");
        return;
    }
    std::shared_ptr<mapit::AbstractEntitydata> ed_a = workspace_->getEntitydataForReadWrite(entity_name);
    if ( 0 != std::strcmp(ed_a->type(), TfEntitydata::TYPENAME()) ) {
      log_error("RegistrationStorageHelper: can't add tf, retrieved entity is not of type TfEntitydata");
      throw MAPIT_STATUS_ERROR;
    }
    std::shared_ptr<TfEntitydata> ed_tf = std::static_pointer_cast<TfEntitydata>(ed_a);
    std::shared_ptr<tf::store::TransformStampedList> ed_d = ed_tf->getData();
    if (ed_d == nullptr) {
        log_warn("RegistrationStorageHelper: transforms in entity are empty, do not remove anything");
        return;
    }

    // delete transforms
    ed_d->delete_TransformStamped(stamp_start, stamp_end);

    // store to mapit
    ed_tf->setData(ed_d);
}

void
mapit::RegistrationStorageHelper::operate_pairwise(std::function<bool(  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
                                                                      , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>&
                                                                      , pcl::PointCloud<pcl::PointXYZ>&
                                                                      , Eigen::Affine3f&
                                                                      , double&)> algorithm)
{
    // get target cloud
    mapit::time::Stamp target_stamp;
    pcl::PCLHeader target_header;
    std::shared_ptr<PointcloudEntitydata> entitydata_target;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target_pc;
    target_pc = get_pointcloud( cfg_target_, target_stamp, target_header, entitydata_target );

    // in case of tf-combine, a list is needed
    std::list<std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>>> tf_combine_list;

    for (std::string cfg_input_one : cfg_input_) {
        // get input cloud
        mapit::time::Stamp input_stamp;
        pcl::PCLHeader input_header;
        std::shared_ptr<PointcloudEntitydata> entitydata_input;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_pc;
        input_pc = get_pointcloud( cfg_input_one, input_stamp, input_header, entitydata_input );

        // execute algorithm
        double fitness_score = std::numeric_limits<double>::max();

        Eigen::Affine3f result_transform;
        pcl::PointCloud<pcl::PointXYZ> result_pc;

        bool has_converged = algorithm(input_pc, target_pc, result_pc, result_transform, fitness_score);

        if ( ! has_converged ) {
          log_error("RegistrationStorageHelper: algorithm didn't converged");
          throw MAPIT_STATUS_ERROR;
        } else {
            log_info("RegistrationStorageHelper: matching for cloud \"" + cfg_input_one + "\" to \"" + cfg_target_
                   + "\" finished with fitness score " + std::to_string( fitness_score ));
        }

        // handle the result
        switch (cfg_handle_result_) {
            case RegistrationStorageHelper::HandleResult::data_change: {
                log_info("RegistrationStorageHelper: change pointcloud " + cfg_input_one/* + " with tf:"*/);
    //                std::cout << transform.matrix() << std::endl;
                log_warn("RegistrationStorageHelper: only XYZ will survive, intensity and color will be lost");
                // TODO find way to transform pointcloud2 so that all data survive
                std::shared_ptr<pcl::PCLPointCloud2> icp_out2 = std::make_shared<pcl::PCLPointCloud2>();
                pcl::toPCLPointCloud2(result_pc, *icp_out2);
                icp_out2->header = input_header;
                entitydata_input->setData(icp_out2);
                break;
            }
            case RegistrationStorageHelper::HandleResult::tf_add: {
                mapit_add_tf(input_stamp, result_transform);
                break;
            }
            case RegistrationStorageHelper::HandleResult::tf_combine: {
                // get old tf from buffer
                Eigen::Affine3f tf_in_buffer = Eigen::Affine3f::Identity();
                mapit::time::Stamp stamp = input_stamp;
                unsigned long sec, nsec;
                mapit::time::to_sec_and_nsec(stamp, sec, nsec);
                try {
                    tf::TransformStamped tf = tf_buffer_->lookupTransform(cfg_tf_frame_id_, cfg_tf_child_frame_id_, stamp);
                    tf_in_buffer.translation() << tf.transform.translation.x(), tf.transform.translation.y(), tf.transform.translation.z();
                    tf_in_buffer.rotate( tf.transform.rotation );
                } catch (...) {
                    log_warn("RegistrationStorageHelper: tf \""
                           + cfg_tf_frame_id_ + "\" to \"" + cfg_tf_child_frame_id_
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
                log_error("RegistrationStorageHelper: do not handle result of ICP (no effect), its not yet implemented.");
            }
        }
    }

    // change tfs in mapit
    if ( cfg_handle_result_ == RegistrationStorageHelper::HandleResult::tf_combine) {
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
            tf::TransformStamped tf = tf_buffer_->lookupTransform(cfg_tf_frame_id_, cfg_tf_child_frame_id_, earliest - small_time);
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
            tf::TransformStamped tf = tf_buffer_->lookupTransform(cfg_tf_frame_id_, cfg_tf_child_frame_id_, latest + small_time);
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
        mapit_remove_tfs(earliest, latest);

        // add new tfs
        for (std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>> tf_to_add : tf_combine_list) {
            mapit_add_tf(tf_to_add.first, *tf_to_add.second);
        }
    }
}


void
mapit::RegistrationStorageHelper::operate_global(  std::function<void(pcl::PointCloud<pcl::PointXYZ>::Ptr)> callback_add_pointcloud
                                                 , std::function<void()> callback_search_and_process_loops
                                                 , std::function<void()> callback_execute_algorithm)
{
    // in case of tf-combine, a list is needed
    std::list<std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>>> tf_combine_list;

    for (std::string cfg_input_one : cfg_input_) {
        // get input cloud
        mapit::time::Stamp input_stamp;
        pcl::PCLHeader input_header;
        std::shared_ptr<PointcloudEntitydata> entitydata_input;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_pc;
        input_pc = get_pointcloud( cfg_input_one, input_stamp, input_header, entitydata_input );

        callback_add_pointcloud(input_pc);
    }

    callback_search_and_process_loops();

    callback_execute_algorithm();

//    {
//        if ( ! has_converged ) {
//          log_error("RegistrationStorageHelper: algorithm didn't converged");
//          throw MAPIT_STATUS_ERROR;
//        } else {
//            log_info("RegistrationStorageHelper: matching for cloud \"" + cfg_input_one + "\" to \"" + cfg_target_
//                   + "\" finished with fitness score " + std::to_string( fitness_score ));
//        }

//        // handle the result
//        switch (cfg_handle_result_) {
//            case RegistrationStorageHelper::HandleResult::data_change: {
//                log_info("RegistrationStorageHelper: change pointcloud " + cfg_input_one/* + " with tf:"*/);
//    //                std::cout << transform.matrix() << std::endl;
//                log_warn("RegistrationStorageHelper: only XYZ will survive, intensity and color will be lost");
//                // TODO find way to transform pointcloud2 so that all data survive
//                std::shared_ptr<pcl::PCLPointCloud2> icp_out2 = std::make_shared<pcl::PCLPointCloud2>();
//                pcl::toPCLPointCloud2(result_pc, *icp_out2);
//                icp_out2->header = input_header;
//                entitydata_input->setData(icp_out2);
//                break;
//            }
//            case RegistrationStorageHelper::HandleResult::tf_add: {
//                mapit_add_tf(input_stamp, result_transform);
//                break;
//            }
//            case RegistrationStorageHelper::HandleResult::tf_combine: {
//                // get old tf from buffer
//                Eigen::Affine3f tf_in_buffer = Eigen::Affine3f::Identity();
//                mapit::time::Stamp stamp = input_stamp;
//                unsigned long sec, nsec;
//                mapit::time::to_sec_and_nsec(stamp, sec, nsec);
//                try {
//                    tf::TransformStamped tf = tf_buffer_->lookupTransform(cfg_tf_frame_id_, cfg_tf_child_frame_id_, stamp);
//                    tf_in_buffer.translation() << tf.transform.translation.x(), tf.transform.translation.y(), tf.transform.translation.z();
//                    tf_in_buffer.rotate( tf.transform.rotation );
//                } catch (...) {
//                    log_warn("RegistrationStorageHelper: tf \""
//                           + cfg_tf_frame_id_ + "\" to \"" + cfg_tf_child_frame_id_
//                           + "\" at time " + std::to_string(sec) + "." + std::to_string(nsec) + " does not exists. Identity will be used");
//                    tf_in_buffer = Eigen::Affine3f::Identity();
//                }
//                // combine tf with tf currently in buffer
//                std::shared_ptr<Eigen::Affine3f> tf_combined = std::shared_ptr<Eigen::Affine3f>(new Eigen::Affine3f(tf_in_buffer * result_transform)); // std::make_shared is not possible because its call by value and that does not work with eigen

//                // store tf to list
//                std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>> tf_pair(stamp, tf_combined);
//                tf_combine_list.push_back(tf_pair);
//                break;
//            }
//            default: {
//                log_error("RegistrationStorageHelper: do not handle result of ICP (no effect), its not yet implemented.");
//            }
//        }
//    }

//    // change tfs in mapit
//    if ( cfg_handle_result_ == RegistrationStorageHelper::HandleResult::tf_combine) {
//        // get time of input clouds
//        mapit::time::Stamp earliest = tf_combine_list.front().first;
//        mapit::time::Stamp latest = tf_combine_list.front().first;
//        mapit::time::nanoseconds small_time = mapit::time::nanoseconds(1); // smallest time we can have
//        for (std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>> tf_time : tf_combine_list) {
//            if (earliest > tf_time.first) {
//                earliest = tf_time.first;
//            }
//            if (latest < tf_time.first) {
//                latest = tf_time.first;
//            }
//        }
//        // get tf just before we change anything, and add that to the list to store, therefore we do not interpolate between old and new tfs
//        try {
//            tf::TransformStamped tf = tf_buffer_->lookupTransform(cfg_tf_frame_id_, cfg_tf_child_frame_id_, earliest - small_time);
//            Eigen::Affine3f tf_in_buffer = Eigen::Affine3f::Identity();
//            tf_in_buffer.translation() << tf.transform.translation.x(), tf.transform.translation.y(), tf.transform.translation.z();
//            tf_in_buffer.rotate( tf.transform.rotation );
//            // store tf to list
//            std::shared_ptr<Eigen::Affine3f> tf_in_buffer_ptr = std::shared_ptr<Eigen::Affine3f>(new Eigen::Affine3f(tf_in_buffer)); // std::make_shared is not possible because its call by value and that does not work with eigen
//            std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>> tf_pair(earliest - small_time, tf_in_buffer_ptr);
//            tf_combine_list.push_back(tf_pair);
//        } catch(...) {
//            // when it doesn't work, there is also nothing todo here, so no worries
//        }

//        // get tf just after we change anything, and add that to the list to store, therefore we do not interpolate between old and new tfs
//        try {
//            tf::TransformStamped tf = tf_buffer_->lookupTransform(cfg_tf_frame_id_, cfg_tf_child_frame_id_, latest + small_time);
//            Eigen::Affine3f tf_in_buffer = Eigen::Affine3f::Identity();
//            tf_in_buffer.translation() << tf.transform.translation.x(), tf.transform.translation.y(), tf.transform.translation.z();
//            tf_in_buffer.rotate( tf.transform.rotation );
//            // store tf to list
//            std::shared_ptr<Eigen::Affine3f> tf_in_buffer_ptr = std::shared_ptr<Eigen::Affine3f>(new Eigen::Affine3f(tf_in_buffer)); // std::make_shared is not possible because its call by value and that does not work with eigen
//            std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>> tf_pair(latest + small_time, tf_in_buffer_ptr);
//            tf_combine_list.push_back(tf_pair);
//        } catch(...) {
//            // when it doesn't work, there is also nothing todo here, so no worries
//        }

//        // delete tfs between time of clouds
//        mapit_remove_tfs(earliest, latest);

//        // add new tfs
//        for (std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>> tf_to_add : tf_combine_list) {
//            mapit_add_tf(tf_to_add.first, *tf_to_add.second);
//        }
//    }
}
