#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/layertypes/pose_path.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <iostream>
#include <memory>
#include <upns/errorcodes.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <QFile>

using namespace mapit::msgs;

upns::StatusCode operate_edit_entity(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();

    bool frameIdExists = params["frame_id"].isString();
    std::string frameId;
    if(frameIdExists)
    {
        frameId = params["frame_id"].toString().toStdString();
    }
    bool stampExists = params["stamp"].isObject();
    QJsonObject stamp;
    int sec;
    int nsec;
    if(stampExists)
    {
        stamp = params["stamp"].toObject();
        sec = stamp["sec"].toInt();
        nsec = stamp["nsec"].toInt();
    }
    if(!frameIdExists && !stampExists)
    {
        log_info("No frame_id and no stamp. Nothing to edit.");
        return UPNS_STATUS_OK;
    }

    std::shared_ptr<mapit::msgs::Entity> entity = env->getCheckout()->getEntity(target);
    if(entity == nullptr)
    {
        log_error("Entity not found.");
        return UPNS_STATUS_ERR_DB_NOT_FOUND;
    }
    if(frameIdExists)
    {
        entity->set_frame_id(frameId);
    }
    if(stampExists)
    {
        ::mapit::msgs::Time *stamp = entity->mutable_stamp();
        stamp->set_sec(sec);
        stamp->set_nsec(nsec);
    }
    env->getCheckout()->storeEntity(target, entity);

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Path from JSON File", "fhac", OPERATOR_VERSION, "*", &operate_edit_entity)
