#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/layertypes/tflayer.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <iostream>
#include <memory>
#include <upns/errorcodes.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

// Apply array of transforms.
// {
//    target: "path/to/target.tf"
//    mode: "relative"|"absolute",
//    tf: [
//      0: {mat: [0: m00, 1: m01, m02...], parent: "/path/to/parent", timestamp: unixts},
//      1: {mat: [0: m00_2, ...]},
//      2: ...
//    ]
// }
// relative:
// parent may be a tf or a tree
// if parent is a tree, timestamp is used to choose a appropiate coordinate system (lerp between two tfs).
// if parent is empty, target is used.
// absolute:
// only one tf (index:0) may be given, parent/timestamp must not be used.

template<typename Indexable>
void jsonToMat(float* d, Indexable &json)
{
    //TODO: unroll
    for(int i=0 ; i<16 ; ++i)
    {
        if(json[i].isDouble())
        {
            d[i] = static_cast<float>(json[i].toDouble());
        }
    }
}

upns::StatusCode operate(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();

    bool newlyCreated = false; // In this case, there will be no tf to read
    // Get Target
    std::shared_ptr<mapit::msgs::Entity> tfEntity = env->getCheckout()->getEntity(target);
    if(tfEntity == NULL)
    {
        // If target could not be received, create new entity
        tfEntity = std::shared_ptr<mapit::msgs::Entity>(new mapit::msgs::Entity);
        tfEntity->set_type(TfEntitydata::TYPENAME());
        StatusCode s = env->getCheckout()->storeEntity(target, tfEntity);
        if(!upnsIsOk(s))
        {
            log_error("Failed to create entity.");
            return UPNS_STATUS_ERR_UNKNOWN;
        }
        newlyCreated = true;
    }
    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    if(abstractEntitydata == NULL)
    {
        return UPNS_STATUS_ERR_UNKNOWN;
    }
    std::shared_ptr<TfEntitydata> entityData = std::dynamic_pointer_cast<TfEntitydata>( abstractEntitydata );
    if(entityData == NULL)
    {
        log_error("Tf Transform has wrong type.");
        return UPNS_STATUS_ERR_UNKNOWN;
    }
    TfMatPtr tf;
    if(newlyCreated)
    {
        tf = TfMatPtr(new TfMat);
    }
    else
    {
        tf = entityData->getData();
    }

    std::string mode = params["mode"].toString().toStdString();

    if(mode == "absolute")
    {
        TfMat mat;
        *tf = TfMat::Identity();
        float *d = tf->data();
        if(params["tf"].isObject())
        {
            QJsonObject obj(params["tf"].toObject());
            if(obj["mat"].isArray())
            {
                QJsonArray idxbl(obj["mat"].toArray());
                jsonToMat(d, idxbl);
            }
            else
            {
                log_error("wrong parameter given, absolute tf has no mat");
                return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
            }
        }
        else if(params["tf"].isArray() && params["tf"].toArray()[0].isObject())
        {
            QJsonArray arr(params["tf"].toArray());
            QJsonObject obj(arr[0].toObject());
            if(obj["mat"].isArray())
            {
                QJsonArray idxbl(obj["mat"].toArray());
                jsonToMat(d, idxbl);
            }
            else
            {
                log_error("wrong parameter given, absolute tf[0] has no mat");
                return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
            }
        }
        else
        {
            log_error("wrong parameter given, tf is invalid");
            return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
        }
    }
    else if(mode == "relative")
    {
    }
    else
    {
        log_error("wrong parameter given, mode was not relative or absolute");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    entityData->setData(tf);

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "use pcl normalestimation on a pointcloud", "fhac", OPERATOR_VERSION, "any", &operate)
