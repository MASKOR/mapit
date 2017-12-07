#include "icp.h"

#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/errorcodes.h>
#include <upns/logging.h>

#include <upns/layertypes/tflayer/tf2/buffer_core.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <pcl/conversions.h>

#include <pcl/registration/icp.h>

mapit::ICP::ICP(upns::OperationEnvironment* env, upns::StatusCode &status)
{
    status = UPNS_STATUS_OK;

    // pointcloud parameter
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());
    if        ( params["input"].isString() ) {
        cfg_input_.push_back( params["input"].toString().toStdString() );
    } else if ( params["input"].isArray() ) {
        for (QJsonValue input : params["input"].toArray() ) {
            if ( ! input.isString() ) {
                log_error("reg_local_icp: cfg \"input\" does not is a string or array of strings");
                status = UPNS_STATUS_INVALID_ARGUMENT;
                return;
            }
            cfg_input_.push_back( input.toString().toStdString() );
        }
    } else {
        log_error("reg_local_icp: cfg \"input\" does not is a string or array of strings");
        status = UPNS_STATUS_INVALID_ARGUMENT;
        return;
    }
    cfg_target_ = params["target"].toString().toStdString();
    cfg_input_.remove( cfg_target_ ); // delete the target from the input (in the case it is specified in both)
    std::string output_pointcloud = "reg_local_icp: executing ICP on pointclouds [ ";
    for (std::string cfg_input_one : cfg_input_) {
        output_pointcloud += "\"" + cfg_input_one + "\", ";
    }
    output_pointcloud += "] to pointcloud \"" + cfg_target_ + "\"";
    log_info(output_pointcloud);

    // config parameter
    cfg_use_frame_id_ = ! params["frame_id"].isNull();
    if (cfg_use_frame_id_) {
        cfg_frame_id_ = params["frame_id"].toString().toStdString();
        log_info("reg_local_icp: transform to \"" + cfg_frame_id_ + "\" before executing ICP");
    } else {
        log_info("reg_local_icp: do not transform in any frame before executing ICP");
    }

    cfg_use_metascan_ = params.contains("use-metascan") ? params["use-metascan"].toBool() : false;
    if (cfg_use_metascan_) {
        log_info("reg_local_icp: use metascan on target (with voxelgrid TODO, not yet implemented)");
    } else {
        log_info("reg_local_icp: do not use metascan");
    }

    // handle parameter
    std::string cfg_handle_result_str = params["handle-result"].toString().toStdString();
    if        ( 0 == cfg_handle_result_str.compare("tf-add") ) {
      cfg_handle_result_ = HandleResult::tf_add;
    } else if ( 0 == cfg_handle_result_str.compare("tf-change") ) {
      cfg_handle_result_ = HandleResult::tf_change;
    } else if ( 0 == cfg_handle_result_str.compare("data-change") ) {
      cfg_handle_result_ = HandleResult::data_change;
    } else {
      log_error("reg_local_icp: can't handle parameter \"handle-result\" = \"" + cfg_handle_result_str + "\"");
      status = UPNS_STATUS_INVALID_ARGUMENT;
      return;
    }

    if (cfg_handle_result_ == HandleResult::tf_add
     || cfg_handle_result_ == HandleResult::tf_change) {
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
        log_error("reg_local_icp: \"tf-is_static\" := true is only allowed for one \"input\" cloud specified");
        status = UPNS_STATUS_INVALID_ARGUMENT;
        return;
    }

    // get env
    checkout_ = env->getCheckout();
    cfg_tf_prefix_ = params.contains("tf-prefix") ? params["tf-prefix"].toString().toStdString() : "";

    tf_buffer_ = std::make_shared<mapit::tf2::BufferCore>(checkout_, cfg_tf_prefix_);
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>
mapit::ICP::get_pointcloud(std::string path, upns::StatusCode &status, mapit::time::Stamp& stamp, pcl::PCLHeader& header, std::shared_ptr<PointcloudEntitydata> entitydata)
{
    status = UPNS_STATUS_OK;
    std::shared_ptr<mapit::msgs::Entity> entity = checkout_->getEntity( path );
    if (entity == nullptr) {
      status = UPNS_STATUS_INVALID_ARGUMENT;
      return nullptr;
    }
    std::string frame_id = entity->frame_id();
    unsigned long sec = entity->stamp().sec();
    unsigned long nsec = entity->stamp().nsec();
    stamp = mapit::time::from_sec_and_nsec( sec, nsec );
    std::shared_ptr<AbstractEntitydata> abstract_entitydata = checkout_->getEntitydataForReadWrite( path );
    if ( 0 != std::strcmp( abstract_entitydata->type(), PointcloudEntitydata::TYPENAME() )) {
      status = UPNS_STATUS_INVALID_ARGUMENT;
      return nullptr;
    }
    entitydata = std::static_pointer_cast<PointcloudEntitydata>( abstract_entitydata );
    std::shared_ptr<pcl::PCLPointCloud2> pc2 = entitydata->getData();
    header = pc2->header;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromPCLPointCloud2(*pc2, *pc);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pc_transformed = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (cfg_use_frame_id_) {
      upns::tf::TransformStamped tf = tf_buffer_->lookupTransform(cfg_frame_id_, frame_id, stamp);
      pcl::transformPointCloud(*pc, *pc_transformed, tf.transform.translation.vector(), tf.transform.rotation);
    } else {
      *pc_transformed = *pc;
    }

    return pc_transformed;
}

upns::StatusCode
mapit::ICP::operate()
{
    // get target cloud
    upns::StatusCode status;
    mapit::time::Stamp target_stamp;
    pcl::PCLHeader target_header;
    std::shared_ptr<PointcloudEntitydata> entitydata_target;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target_pc = get_pointcloud( cfg_target_, status, target_stamp, target_header, entitydata_target );
    if ( ! upnsIsOk(status) ) {
        return status;
    }

    for (std::string cfg_input_one : cfg_input_) {
        // get input clouds
        mapit::time::Stamp input_stamp;
        pcl::PCLHeader input_header;
        std::shared_ptr<PointcloudEntitydata> entitydata_input;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_pc = get_pointcloud( cfg_input_one, status, input_stamp, input_header, entitydata_input );
        if ( ! upnsIsOk(status) ) {
            return status;
        }

        // do ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(input_pc);
        icp.setInputTarget(target_pc);
        pcl::PointCloud<pcl::PointXYZ> icp_out;
        icp.align(icp_out);

        if ( ! icp.hasConverged() ) {
          log_error("reg_local_icp: algorithm didn't converged");
          return UPNS_STATUS_ERROR;
        } else {
            log_info("reg_local_icp: ICP for cloud \"" + cfg_input_one + "\" to \"" + cfg_target_
                   + "\" finished with fitness score " + std::to_string(icp.getFitnessScore()));

            if (cfg_use_metascan_) {
                *target_pc += icp_out;
            }
        }

        Eigen::Affine3f transform( icp.getFinalTransformation() );

        // handle the result
        switch (cfg_handle_result_) {
          case HandleResult::data_change: {
            log_info("reg_local_icp: change pointcloud " + cfg_input_one + " with tf:");
            std::cout << transform.matrix() << std::endl;
            log_warn("reg_local_icp: only XYZ will survive, intensity and color will be lost");
            // TODO find way to transform pointcloud2 so that all data survive
            std::shared_ptr<pcl::PCLPointCloud2> icp_out2 = std::make_shared<pcl::PCLPointCloud2>();
            pcl::toPCLPointCloud2(icp_out, *icp_out2);
            icp_out2->header = input_header;
            entitydata_input->setData(icp_out2);
            break;
          }
          case HandleResult::tf_add: {
            unsigned long sec_log, nsec_log;
            mapit::time::to_sec_and_nsec(input_stamp, sec_log, nsec_log);
            log_info("reg_local_icp: add "
                   + (cfg_tf_is_static_ ? "static" : "dynamic")
                   + " transform from \"" + cfg_tf_frame_id_ + "\" to \"" + cfg_tf_child_frame_id_
                   + "\" at time " + std::to_string(sec_log) + "." + std::to_string(nsec_log) );
            std::cout << transform.matrix() << std::endl;
            // get infos
            mapit::time::Stamp stamp = input_stamp;
            std::string entity_name = cfg_tf_prefix_ + "/" + upns::tf::store::TransformStampedList::get_entity_name(cfg_tf_frame_id_, cfg_tf_child_frame_id_);
            // get entity and data
            // TODO how to check for existance of entity?
            std::shared_ptr<mapit::msgs::Entity> entity = checkout_->getEntity( entity_name );
            std::shared_ptr<TfEntitydata> ed_tf;
            if (entity == nullptr) {
              entity = std::make_shared<mapit::msgs::Entity>();
              entity->set_type(TfEntitydata::TYPENAME());
              StatusCode s = checkout_->storeEntity(entity_name, entity);
              if( ! upnsIsOk(s) ) {
                log_error("reg_local_icp: Failed to create entity.");
                return UPNS_STATUS_ERROR;
              }
            }
            std::shared_ptr<upns::AbstractEntitydata> ed_a = checkout_->getEntitydataForReadWrite(entity_name);
            if ( 0 != std::strcmp(ed_a->type(), TfEntitydata::TYPENAME()) ) {
              log_error("reg_local_icp: can't add tf, retrieved entity is not of type TfEntitydata");
              return UPNS_STATUS_ERROR;
            }
            ed_tf = std::static_pointer_cast<TfEntitydata>(ed_a);
            std::shared_ptr<tf::store::TransformStampedList> ed_d = ed_tf->getData();
            if (ed_d == nullptr) {
                ed_d = std::make_shared<tf::store::TransformStampedList>(cfg_tf_frame_id_, cfg_tf_child_frame_id_, cfg_tf_is_static_);
            }

            std::unique_ptr<upns::tf::TransformStamped> tfs = std::make_unique<upns::tf::TransformStamped>();

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
            checkout_->storeEntity(entity_name, entity);
            ed_tf->setData(ed_d);
            break;
          }
          case HandleResult::tf_change: {
            log_info("reg_local_icp: change tf");
            log_error("reg_local_icp: do not handle result of ICP (no effect), its not yet implemented.");
            break;
          }
          default: {
            log_error("reg_local_icp: do not handle result of ICP (no effect), its not yet implemented.");
          }
        }
    }

    return UPNS_STATUS_OK;
}
