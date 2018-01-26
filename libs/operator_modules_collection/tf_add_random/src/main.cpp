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

struct RandomStorage {
    double xMin;
    double xMax;
    double x;
    double yMin;
    double yMax;
    double y;
    double zMin;
    double zMax;
    double z;
    double rollMin;
    double rollMax;
    double roll;
    double pitchMin;
    double pitchMax;
    double pitch;
    double yawMin;
    double yawMax;
    double yaw;
};

double getRandom(const double& min, const double& max)
{
    double randomFloat = (double)rand() / RAND_MAX;
    return min + randomFloat * (max - min);
}

upns::StatusCode saveRandomTFForEntity(  upns::OperationEnvironment* env
                                       , std::shared_ptr<mapit::msgs::Entity> entity
                                       , const std::string& target
                                       , const std::string& frame_id
                                       , const std::string& tfStoragePrefix
                                       , RandomStorage randomStorage)
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
    unsigned long seed = entity->stamp().sec() + entity->stamp().nsec();
    srand( seed ); // !!! this is not random, but we want it repeatable for the same data
    randomStorage.x = getRandom(randomStorage.xMin, randomStorage.xMax);
    randomStorage.y = getRandom(randomStorage.yMin, randomStorage.yMax);
    randomStorage.z = getRandom(randomStorage.zMin, randomStorage.zMax);
    randomStorage.roll  = getRandom(randomStorage.rollMin, randomStorage.rollMax);
    randomStorage.pitch = getRandom(randomStorage.pitchMin, randomStorage.pitchMax);
    randomStorage.yaw   = getRandom(randomStorage.yawMin, randomStorage.yawMax);

    // create tf
    Eigen::Quaternionf tfRot;
    tfRot =   Eigen::AngleAxisf(randomStorage.roll, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(randomStorage.pitch, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(randomStorage.yaw, Eigen::Vector3f::UnitZ());
    std::unique_ptr<upns::tf::TransformStamped> tfStamped = std::make_unique<upns::tf::TransformStamped>();
    tfStamped->frame_id = frame_id;
    tfStamped->child_frame_id = child_frame_id;
    tfStamped->stamp = stamp;
    tfStamped->transform.translation.x() = randomStorage.x;
    tfStamped->transform.translation.y() = randomStorage.y;
    tfStamped->transform.translation.z() = randomStorage.z;
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

double getParam(QJsonObject params, std::string name)
{
    if ( params.find(name.c_str()) != params.end() &&  params[name.c_str()].isDouble() ){
        return params[name.c_str()].toDouble();
    } else {
        log_error("tf_add_noise: param \"" + name + "\" is either not set or not a double");
        throw std::exception();
    }
}

upns::StatusCode operate_tf_add_noise(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    log_info( "tf_add_noise params:" + env->getParameters() );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();
    std::string tfStoragePrefix = params["tf-storage-prefix"].toString().toStdString();
    std::string frame_id = params["frame_id"].toString().toStdString();
    RandomStorage randomStorage;
    try {
        randomStorage.xMin = getParam(params, "x-min");
        randomStorage.xMax = getParam(params, "x-max");
        randomStorage.yMin = getParam(params, "y-min");
        randomStorage.yMax = getParam(params, "y-max");
        randomStorage.zMin = getParam(params, "z-min");
        randomStorage.zMax = getParam(params, "z-max");
        randomStorage.rollMin = getParam(params, "roll-min");
        randomStorage.rollMax = getParam(params, "roll-max");
        randomStorage.pitchMin = getParam(params, "pitch-min");
        randomStorage.pitchMax = getParam(params, "pitch-max");
        randomStorage.yawMin = getParam(params, "yaw-min");
        randomStorage.yawMax = getParam(params, "yaw-max");
    } catch (...) {
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }


    std::shared_ptr<mapit::msgs::Entity> entity = env->getCheckout()->getEntity(target);
    if ( entity ) {
        // execute on entity
        return saveRandomTFForEntity(env, entity, target, frame_id, tfStoragePrefix, randomStorage);
    } else if ( env->getCheckout()->getTree(target) ) {
        // execute on tree
        upns::StatusCode status = UPNS_STATUS_OK;
        env->getCheckout()->depthFirstSearch(
                      target
                    , depthFirstSearchAll(mapit::msgs::Tree)
                    , depthFirstSearchAll(mapit::msgs::Tree)
                    , [&](std::shared_ptr<mapit::msgs::Entity> obj, const ObjectReference& ref, const upns::Path &path)
                        {
                            status = saveRandomTFForEntity(env, obj, path, frame_id, tfStoragePrefix, randomStorage);
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
