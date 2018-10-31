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

#include <thread>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <mutex>

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

    std::string output_pointcloud = "RegistrationStorageHelper: executing algorithm on pointclouds [ ";
    for (std::string cfg_input_one : cfg_input_) {
        output_pointcloud += "\"" + cfg_input_one + "\", ";
    }
    output_pointcloud += "]";
    log_info(output_pointcloud);

    // config parameter
    cfg_use_frame_id_ = ! (params["frame_id"].isNull() || params["frame_id"].toString().isEmpty());
    if (cfg_use_frame_id_) {
        cfg_frame_id_ = params["frame_id"].toString().toStdString();
        log_info("RegistrationStorageHelper: transform to \"" + cfg_frame_id_ + "\" before executing algorithm");
    } else {
        log_info("RegistrationStorageHelper: do not transform in any frame before executing algorithm");
    }

    cfg_parallel_threads_ = std::thread::hardware_concurrency();

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

    cfg_use_poor_mans_prediction_ = params.contains("use_poor_mans_prediction") ? params["use_poor_mans_prediction"].toBool() : false;
    if (cfg_use_poor_mans_prediction_) {
        log_info("RegistrationStorageHelper: use poor man's prediction");
    } else {
        log_info("RegistrationStorageHelper: don't use poor man's prediction");
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
    if (cfg_use_frame_id_) {
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_transformed = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        mapit::tf::TransformStamped tf = tf_buffer_->lookupTransform(cfg_frame_id_, frame_id, stamp);
        pcl::transformPointCloud(*pc, *pc_transformed, tf.transform.translation.vector(), tf.transform.rotation);

        return pc_transformed;
    } else {
        return pc;
    }
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
mapit::RegistrationStorageHelper::operate_pairwise(std::function<bool(  const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
                                                                      , const Eigen::Affine3f& initial_guess_transform
                                                                      , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>&
                                                                      , pcl::PointCloud<pcl::PointXYZ>&
                                                                      , Eigen::Affine3f&
                                                                      , double&)> algorithm
                                                   , const bool& use_metascan)
{
    if (use_metascan) {
        cfg_parallel_threads_ = 1;
    }
    if (cfg_use_poor_mans_prediction_) {
        cfg_parallel_threads_ = 1;
    }
    std::vector<mapit::RegistrationStorageHelper::PCD> pointclouds( cfg_input_.size() );
    for (size_t pc_id = 0; pc_id < cfg_input_.size(); ++pc_id) {
        log_info("RegistrationStorageHelper: load pointcloud " << cfg_input_.at(pc_id));
        pointclouds.at(pc_id).pc = get_pointcloud( cfg_input_.at(pc_id), pointclouds.at(pc_id).stamp, pointclouds.at(pc_id).header, pointclouds.at(pc_id).entitydata );
    }

    // in case of tf add, add identity for the first cloud
    if ( cfg_handle_result_ == RegistrationStorageHelper::HandleResult::tf_add ) {
        mapit::RegistrationStorageHelper::PCD first = pointclouds.at(0);
        Eigen::Affine3f identity = Eigen::Affine3f::Identity();
        mapit_add_tf(first.stamp, identity);
    }

    // in case of tf-combine, a list is needed
    std::list<std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>>> tf_combine_list;

    std::vector<std::shared_ptr<std::thread>> threads( cfg_input_.size() );

    std::vector<Eigen::Affine3f> pair_transform( cfg_input_.size(), Eigen::Affine3f::Identity() );
    threads.at(0) = std::make_shared<std::thread>([](){
        // there is no thread for the first pointcloud therefore create a special thread that can be joint
    });

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target_pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if ( use_metascan ) {
        mapit::RegistrationStorageHelper::PCD target = pointclouds.at(0);
        *target_pc = *target.pc;
    }
    boost::interprocess::interprocess_semaphore thread_semaphore(cfg_parallel_threads_);
    std::mutex mutex;
    log_info("RegistrationStorageHelper: run with " << cfg_parallel_threads_ << " parallel threads");
    for (size_t pc_id = 1; pc_id < cfg_input_.size(); ++pc_id) {
        thread_semaphore.wait(); // only start #CPUs ICP computations at once
        threads.at(pc_id) = std::make_shared<std::thread>([ &thread_semaphore
                                                          , &mutex
                                                          , this
                                                          , pc_id
                                                          , &use_metascan
                                                          , &target_pc
                                                          , &pointclouds
                                                          , &tf_combine_list
                                                          , &threads
                                                          , &algorithm
                                                          , &pair_transform](){
            std::string target_name = cfg_input_.at(pc_id-1);
            std::string input_name = cfg_input_.at(pc_id);

            // get target cloud
            mapit::RegistrationStorageHelper::PCD target = pointclouds.at(pc_id-1);
            if ( ! use_metascan) {
                target_pc = target.pc;
            }

            // get input cloud
            mapit::RegistrationStorageHelper::PCD input = pointclouds.at(pc_id);

            // execute algorithm
            double fitness_score = std::numeric_limits<double>::max();

            Eigen::Affine3f result_transform;
            pcl::PointCloud<pcl::PointXYZ> result_pc;

            Eigen::Affine3f initial_guess_transform = Eigen::Affine3f::Identity();
            if ( ! use_metascan && cfg_use_poor_mans_prediction_ && pc_id >= 2) {
//                initial_guess_transform = (pair_transform.at(pc_id-2).inverse() * pair_transform.at(pc_id-1)).matrix();
                float factor = 1.f / 4;
                Eigen::Affine3f previous_transform = (pair_transform.at(pc_id-2).inverse() * pair_transform.at(pc_id-1));
                initial_guess_transform.matrix()(0, 3) = previous_transform.matrix()(0, 3) * factor;
                initial_guess_transform.matrix()(1, 3) = previous_transform.matrix()(1, 3) * factor;
                initial_guess_transform.matrix()(2, 3) = previous_transform.matrix()(2, 3) * factor;

//                log_info("RegistrationStorageHelper: using initial guess:\n" << initial_guess_transform.matrix());
            }

            bool has_converged = algorithm(input.pc, initial_guess_transform, target_pc, result_pc, result_transform, fitness_score);

            if ( ! has_converged ) {
                log_error("RegistrationStorageHelper: algorithm didn't converged for " << input_name << " -> " << target_name);
              // TODO
    //          initial_guess = Eigen::Affine3f::Identity();
                throw MAPIT_STATUS_ERROR;
                threads.at(pc_id-1)->join();
                pair_transform.at(pc_id) = pair_transform.at(pc_id-1);
            } else {
                // TODO wait for threads.at(pc_id-1) != nullptr
                log_info("RegistrationStorageHelper: matching for cloud \"" + input_name + "\" to \"" + target_name
                       + "\" finished with fitness score " + std::to_string( fitness_score ));
                thread_semaphore.post(); // see this thread as finished and start the next one
                threads.at(pc_id-1)->join();

                if ( use_metascan) {
                    pair_transform.at(pc_id) = result_transform;
                } else {
                    pair_transform.at(pc_id) = pair_transform.at(pc_id-1) * result_transform; // This way we can parallel the registration
                }
                result_transform = pair_transform.at(pc_id);
            }

            // handle the result
            mutex.lock();
            switch (cfg_handle_result_) {
                case RegistrationStorageHelper::HandleResult::data_change: {
                    log_info("RegistrationStorageHelper: change pointcloud " + input_name/* + " with tf:"*/);
        //                std::cout << transform.matrix() << std::endl;
                    log_warn("RegistrationStorageHelper: only XYZ will survive, intensity and color will be lost");
                    // TODO find way to transform pointcloud2 so that all data survive
                    std::shared_ptr<pcl::PCLPointCloud2> icp_out2 = std::make_shared<pcl::PCLPointCloud2>();
                    pcl::toPCLPointCloud2(result_pc, *icp_out2);
                    icp_out2->header = input.header;
                    input.entitydata->setData(icp_out2);
                    break;
                }
                case RegistrationStorageHelper::HandleResult::tf_add: {
                    mapit_add_tf(input.stamp, result_transform);
                    break;
                }
                case RegistrationStorageHelper::HandleResult::tf_combine: {
                    // get old tf from buffer
                    Eigen::Affine3f tf_in_buffer = Eigen::Affine3f::Identity();
                    mapit::time::Stamp stamp = input.stamp;
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
            mutex.unlock();
        });
    }
    // Wait for last thread
    threads.at( cfg_input_.size()-1 )->join();
    log_info("RegistrationStorageHelper: everthing is done");

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
                                                 , std::function<void(  HandleResult&
                                                                      , std::vector<Eigen::Affine3f>&
                                                                      , std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>&
                                                                      , pcl::PointCloud<pcl::PointXYZ>::Ptr)> callback_execute_algorithm)
{
    // in case of tf-combine, a list is needed
    std::list<std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>>> tf_combine_list;

    log_info("RegistrationStorageHelper: load pointclouds");
    for (std::string cfg_input_one : cfg_input_) {
        // get input cloud
        mapit::time::Stamp input_stamp;
        pcl::PCLHeader input_header;
        std::shared_ptr<PointcloudEntitydata> entitydata_input;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_pc;
        input_pc = get_pointcloud( cfg_input_one, input_stamp, input_header, entitydata_input );

        log_info("RegistrationStorageHelper:\t load pointcloud " << cfg_input_one);
        callback_add_pointcloud(input_pc);
    }

    log_info("RegistrationStorageHelper: start to search for loops");
    callback_search_and_process_loops();

    log_info("RegistrationStorageHelper: execute algorithm");
    std::vector<Eigen::Affine3f> tfs;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointclouds;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud;
    callback_execute_algorithm(cfg_handle_result_, tfs, pointclouds, pointcloud);

    log_info("RegistrationStorageHelper: process result");
//    if ( ! has_converged ) {
//      log_error("RegistrationStorageHelper: algorithm didn't converged");
//      throw MAPIT_STATUS_ERROR;
//    } else {
//        log_info("RegistrationStorageHelper: matching for cloud \"" + cfg_input_one + "\" to \"" + cfg_target_
//               + "\" finished with fitness score " + std::to_string( fitness_score ));
//    }

    // handle the result
    for (size_t i = 0; i < cfg_input_.size(); ++i) {
        std::string cfg_input_one = cfg_input_.at(i);

        // get entity
        mapit::time::Stamp input_stamp;
        pcl::PCLHeader input_header;
        std::shared_ptr<PointcloudEntitydata> entitydata;
        get_pointcloud( cfg_input_one, input_stamp, input_header, entitydata );

        switch (cfg_handle_result_) {
            case RegistrationStorageHelper::HandleResult::data_change: {
                pcl::PointCloud<pcl::PointXYZ>::Ptr result_pc = pointclouds.at(i);
                log_info("RegistrationStorageHelper: change pointcloud " + cfg_input_one);
                log_warn("RegistrationStorageHelper: only XYZ will survive, intensity and color will be lost");
                // TODO find way to transform pointcloud2 so that all data survive

                // convert to pcl::PCLPointCloud2
                std::shared_ptr<pcl::PCLPointCloud2> icp_out2 = std::make_shared<pcl::PCLPointCloud2>();
                pcl::toPCLPointCloud2(*result_pc, *icp_out2);
                icp_out2->header = input_header;

                // write entity data
                entitydata->setData(icp_out2);
                break;
            }
            case RegistrationStorageHelper::HandleResult::tf_add: {
                mapit_add_tf(input_stamp, tfs.at(i));
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
                std::shared_ptr<Eigen::Affine3f> tf_combined = std::shared_ptr<Eigen::Affine3f>(new Eigen::Affine3f(tf_in_buffer * tfs.at(i))); // std::make_shared is not possible because its call by value and that does not work with eigen

                // store tf to list
                std::pair<mapit::time::Stamp, std::shared_ptr<Eigen::Affine3f>> tf_pair(stamp, tf_combined);
                tf_combine_list.push_back(tf_pair);
                break;
            }
            default: {
                log_error("RegistrationStorageHelper: do not handle result of global registration, its not yet implemented. :'(");
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
