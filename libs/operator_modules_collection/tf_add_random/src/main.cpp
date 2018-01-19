#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/layertypes/tflayer.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <iostream>
#include <upns/errorcodes.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/depthfirstsearch.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <mapit/time/time.h>
#include <upns/layertypes/tflayer.h>

upns::StatusCode saveRandomTFForEntity(  upns::OperationEnvironment* env
                                       , std::shared_ptr<mapit::msgs::Entity> entity
                                       , const std::string& target
                                       , const std::string& frame_id
                                       , const std::string& tfStoragePrefix)
{
    if(entity == nullptr) {
        log_error("Can't acces entity: " + target);
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    std::string child_frame_id = entity->frame_id();
    if (child_frame_id.empty()) {
        log_error("Entity " + target + " does not have a frame_id set");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    mapit::time::Stamp stamp = mapit::time::from_msg( entity->stamp() );

    // calculate random
    float x = 0;
    float y = 0;
    float z = 0;
    float roll = 0;
    float pitch = 0;
    float yaw = 0;

    // create tf
    Eigen::Quaternionf tfRot;
    tfRot =   Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    std::unique_ptr<upns::tf::TransformStamped> tfStamped = std::make_unique<upns::tf::TransformStamped>();
    tfStamped->frame_id = frame_id;
    tfStamped->child_frame_id = child_frame_id;
    tfStamped->stamp = stamp;
    tfStamped->transform.translation.x() = x;
    tfStamped->transform.translation.y() = y;
    tfStamped->transform.translation.z() = z;
    tfStamped->transform.rotation = tfRot;

    // get or create tf entity
    std::shared_ptr<mapit::msgs::Entity> e;
    std::shared_ptr<TfEntitydata> ed;
    std::shared_ptr<tf::store::TransformStampedList> edTFList;
    StatusCode s = upns::tf::store::getOrCreateTransformStampedList(env->getCheckout(), frame_id, child_frame_id, tfStoragePrefix, e, ed, edTFList, false);
    if( ! upnsIsOk(s) ) {
      return s;
    }
    edTFList->add_TransformStamped(std::move(tfStamped), false);
    ed->setData(edTFList);
    std::string eName = tfStoragePrefix + "/" + upns::tf::store::TransformStampedList::get_entity_name(frame_id, child_frame_id);
    e->set_frame_id(frame_id);
    *e->mutable_stamp() = mapit::time::to_msg(edTFList->get_stamp_earliest());
    env->getCheckout()->storeEntity(eName, e);

    return UPNS_STATUS_OK;
}

upns::StatusCode operate_tf_add_noise(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    log_info( "tf_add_noise params:" + env->getParameters() );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();
    std::string tfStoragePrefix = params["tf-storage-prefix"].toString().toStdString();
    std::string frame_id = params["frame_id"].toString().toStdString();

    std::shared_ptr<mapit::msgs::Entity> entity = env->getCheckout()->getEntity(target);
    if ( entity ) {
        // execute on entity
        return saveRandomTFForEntity(env, entity, target, frame_id, tfStoragePrefix);
    } else if ( env->getCheckout()->getTree(target) ) {
        // execute on tree
        upns::StatusCode status = UPNS_STATUS_OK;
        env->getCheckout()->depthFirstSearch(
                      target
                    , depthFirstSearchAll(mapit::msgs::Tree)
                    , depthFirstSearchAll(mapit::msgs::Tree)
                    , [&](std::shared_ptr<mapit::msgs::Entity> obj, const ObjectReference& ref, const upns::Path &path)
                        {
                            status = saveRandomTFForEntity(env, obj, path, frame_id, tfStoragePrefix);
                            if ( ! upnsIsOk(status) ) {
                                return false;
                            }
                            return true;
                        }
                    , depthFirstSearchAll(mapit::msgs::Entity)
                    );
        return status;
    } else {
        log_error("operator voxelgrid: target is neither a tree nor entity");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
}

UPNS_MODULE(OPERATOR_NAME, "add noise to transforms", "fhac", OPERATOR_VERSION, TfEntitydata_TYPENAME, &operate_tf_add_noise)
