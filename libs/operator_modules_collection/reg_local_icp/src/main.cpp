#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/errorcodes.h>
#include <string>

#include <upns/layertypes/tflayer.h>
#include <upns/layertypes/tflayer/utils.h>
#include <upns/layertypes/tflayer/tf2/buffer_core.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <pcl/conversions.h>

#include <pcl/registration/icp.h>

#include "icp.h"

upns::StatusCode operate_reg_local_icp(upns::OperationEnvironment* env)
{
    /** structure:
     * {
     *  <string>"tf_prefix" : ..., // the prefix where to look for transforms (default "")
     *  <string>"input"[] : ..., // can either be, a list of entities or a tree containing enteties
     *  <string>"target" : ...,
     *  optional <string>"frame_id" : ..., // all data given to ICP will be in this frame
     *  <enum-as-string>"handle-result" : ["tf-add", "tf-change", "data-change"],
     *  optional <string>"store-prefix" : ..., // in case of tf change or add (default "")
     *  optional <string>"tf-frame_id" : ..., // in case of tf change or add
     *  optional <string>"tf-child_frame_id" : ..., // in case of tf change or add
     * }
     */
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());
    std::list<std::string> cfg_input;
    if        ( params["input"].isString() ) {
        cfg_input.push_back( params["input"].toString().toStdString() );
    } else if ( params["input"].isArray() ) {
        for (QJsonValue input : params["input"].toArray() ) {
            if ( ! input.isString() ) {
                log_error("reg_local_icp: cfg \"input\" does not is a string or array of strings");
                return UPNS_STATUS_INVALID_ARGUMENT;
            }
            cfg_input.push_back( input.toString().toStdString() );
        }
    } else {
        log_error("reg_local_icp: cfg \"input\" does not is a string or array of strings");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    std::string cfg_target = params["target"].toString().toStdString();
    bool cfg_use_frame_id = ! params["frame_id"].isNull();
    std::string cfg_frame_id;
    if (cfg_use_frame_id) {
      cfg_frame_id = params["frame_id"].toString().toStdString();
    }
    enum class HandleResult {tf_add, tf_change, data_change};
    HandleResult cfg_handle_result;
    std::string cfg_handle_result_str = params["handle-result"].toString().toStdString();
    if        ( 0 == cfg_handle_result_str.compare("tf-add") ) {
      cfg_handle_result = HandleResult::tf_add;
    } else if ( 0 == cfg_handle_result_str.compare("tf-change") ) {
      cfg_handle_result = HandleResult::tf_change;
    } else if ( 0 == cfg_handle_result_str.compare("data-change") ) {
      cfg_handle_result = HandleResult::data_change;
    } else {
      log_error("reg_local_icp: can't handle parameter \"handle-result\" = \"" + cfg_handle_result_str + "\"");
      return UPNS_STATUS_INVALID_ARGUMENT;
    }

    std::string cfg_tf_frame_id = "not-set";
    std::string cfg_tf_child_frame_id = "not-set";
    std::string cfg_store_prefix = "";
    if (cfg_handle_result == HandleResult::tf_add
     || cfg_handle_result == HandleResult::tf_change) {
      cfg_tf_frame_id = params["tf-frame_id"].toString().toStdString();
      cfg_tf_child_frame_id = params["tf-child_frame_id"].toString().toStdString();
      if ( params.contains("store-prefix") ) {
        cfg_store_prefix = params["store-prefix"].toString().toStdString();
      }
    }

    CheckoutRaw* checkout = env->getCheckout();
    std::string cfg_tf_prefix = params.contains("tf_prefix") ? params["tf_prefix"].toString().toStdString() : "";
    mapit::tf2::BufferCore tf_buffer(checkout, cfg_tf_prefix);

    // get input clouds
    std::shared_ptr<mapit::msgs::Entity> entity_input = checkout->getEntity( cfg_input.front() );
    if (entity_input == nullptr) {
      return UPNS_STATUS_INVALID_ARGUMENT;
    }
    std::string input_frame_id = entity_input->frame_id();
    mapit::time::Stamp input_stamp = mapit::time::from_sec_and_nsec( entity_input->stamp().sec(), entity_input->stamp().nsec() );
    std::shared_ptr<AbstractEntitydata> abstract_entitydata_input = checkout->getEntitydataForReadWrite( cfg_input.front() );
    if ( 0 != std::strcmp( abstract_entitydata_input->type(), PointcloudEntitydata::TYPENAME() )) {
      return UPNS_STATUS_INVALID_ARGUMENT;
    }
    std::shared_ptr<PointcloudEntitydata> entitydata_input = std::static_pointer_cast<PointcloudEntitydata>( abstract_entitydata_input );
    std::shared_ptr<pcl::PCLPointCloud2> input_pc2 = entitydata_input->getData();
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromPCLPointCloud2(*input_pc2, *input_pc);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input_pc_transformed = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (cfg_use_frame_id) {
      upns::tf::TransformStamped input_tf = tf_buffer.lookupTransform(cfg_frame_id, input_frame_id, input_stamp);
      pcl::transformPointCloud(*input_pc, *input_pc_transformed, input_tf.transform.translation.vector(), input_tf.transform.rotation);
    } else {
      *input_pc_transformed = *input_pc;
    }

    // get target cloud
    std::shared_ptr<mapit::msgs::Entity> entity_target = checkout->getEntity( cfg_input.front() );
    if (entity_target == nullptr) {
      return UPNS_STATUS_INVALID_ARGUMENT;
    }
    std::string target_frame_id = entity_target->frame_id();
    mapit::time::Stamp target_stamp = mapit::time::from_sec_and_nsec( entity_target->stamp().sec(), entity_target->stamp().nsec() );
    std::shared_ptr<AbstractEntitydata> abstract_entitydata_target = checkout->getEntitydataForReadWrite( cfg_target );
    if ( 0 != std::strcmp( abstract_entitydata_target->type(), PointcloudEntitydata::TYPENAME() )) {
      return UPNS_STATUS_INVALID_ARGUMENT;
    }
    std::shared_ptr<PointcloudEntitydata> entitydata_target = std::static_pointer_cast<PointcloudEntitydata>( abstract_entitydata_target );
    std::shared_ptr<pcl::PCLPointCloud2> target_pc2 = entitydata_target->getData();
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target_pc = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromPCLPointCloud2(*target_pc2, *target_pc);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target_pc_transformed = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (cfg_use_frame_id) {
      upns::tf::TransformStamped target_tf = tf_buffer.lookupTransform(cfg_frame_id, target_frame_id, target_stamp);
      pcl::transformPointCloud(*target_pc, *target_pc_transformed, target_tf.transform.translation.vector(), target_tf.transform.rotation);
    } else {
      *target_pc_transformed = *target_pc;
    }

    // do ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(input_pc_transformed);
    icp.setInputTarget(target_pc_transformed);
    pcl::PointCloud<pcl::PointXYZ> icp_out;
    icp.align(icp_out);

    if ( ! icp.hasConverged() ) {
      log_error("reg_local_icp: algorithm didn't converged");
      return UPNS_STATUS_ERROR;
    }

    Eigen::Affine3f transform( icp.getFinalTransformation() );
    log_info("reg_local_icp: finished");
    std::cout << transform.matrix() << std::endl;

    // handle the result
    switch (cfg_handle_result) {
      case HandleResult::data_change: {
        log_info("reg_local_icp: change data");
        log_warn("reg_local_icp: only XYZ will survive, intensity and color will be lost");
        // TODO find way to transform pointcloud2 so that all data survive
        std::shared_ptr<pcl::PCLPointCloud2> icp_out2 = std::make_shared<pcl::PCLPointCloud2>();
        pcl::fromPCLPointCloud2(*target_pc2, *target_pc);
        pcl::toPCLPointCloud2(icp_out, *icp_out2);
        icp_out2->header = input_pc2->header;
        entitydata_input->setData(icp_out2);
        break;
      }
      case HandleResult::tf_add: {
        log_info("reg_local_icp: add tf");
        // get infos
        mapit::time::Stamp stamp = input_stamp;
        std::string entity_name = cfg_store_prefix + "/" + upns::tf::store::TransformStampedList::get_entity_name(cfg_tf_frame_id, cfg_tf_child_frame_id);
        // get entity and data
        // TODO how to check for existance of entity?
        std::shared_ptr<mapit::msgs::Entity> entity = checkout->getEntity( entity_name );
        std::shared_ptr<TfEntitydata> ed_tf;
        if (entity == nullptr) {
          entity = std::make_shared<mapit::msgs::Entity>();
          entity->set_type(TfEntitydata::TYPENAME());
          StatusCode s = checkout->storeEntity(entity_name, entity);
          if( ! upnsIsOk(s) ) {
            log_error("reg_local_icp: Failed to create entity.");
            return UPNS_STATUS_ERROR;
          }
        }
        std::shared_ptr<upns::AbstractEntitydata> ed_a = checkout->getEntitydataForReadWrite(entity_name);
        if ( 0 != std::strcmp(ed_a->type(), TfEntitydata::TYPENAME()) ) {
          log_error("reg_local_icp: can't add tf, retrieved entity is not of type TfEntitydata");
          return UPNS_STATUS_ERROR;
        }
        ed_tf = std::static_pointer_cast<TfEntitydata>(ed_a);
        std::shared_ptr<tf::store::TransformStampedList> ed_d = ed_tf->getData();
        if (ed_d == nullptr) {
            ed_d = std::make_shared<tf::store::TransformStampedList>(cfg_tf_frame_id, cfg_tf_child_frame_id, false);
        }

        std::unique_ptr<upns::tf::TransformStamped> tfs = std::make_unique<upns::tf::TransformStamped>();

        // add data
        tfs->frame_id = cfg_tf_frame_id;
        tfs->child_frame_id = cfg_tf_child_frame_id;
        tfs->stamp = stamp;
        tfs->transform.translation.translation() = transform.translation();
        tfs->transform.rotation = transform.rotation();
        ed_d->add_TransformStamped( std::move(tfs), false);

        unsigned long sec, nsec;
        mapit::time::to_sec_and_nsec(stamp, sec, nsec);
        entity->set_frame_id( cfg_tf_frame_id );
        entity->mutable_stamp()->set_sec( sec );
        entity->mutable_stamp()->set_nsec( nsec );

        // write data
        checkout->storeEntity(entity_name, entity);
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

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "execute icp on pointclouds", "fhac", OPERATOR_VERSION, "any", &operate_reg_local_icp)
