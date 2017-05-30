#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/layertypes/primitive.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <iostream>
#include <memory>
#include <upns/errorcodes.h>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>

using namespace mapit::msgs;

mapit::msgs::Primitive::PrimitiveType stringToType(std::string &type)
{
    if(type.compare("SPHERE") == 0) {
        return mapit::msgs::Primitive::SPHERE;
    } else if(type.compare("CUBE") == 0) {
        return mapit::msgs::Primitive::CUBE;
    } else if(type.compare("PLANE") == 0) {
        return mapit::msgs::Primitive::PLANE;
    } else if(type.compare("CYLINDER") == 0) {
        return mapit::msgs::Primitive::CYLINDER;
    } else if(type.compare("CONE") == 0) {
        return mapit::msgs::Primitive::CONE;
    } else if(type.compare("CAPSULE") == 0) {
        return mapit::msgs::Primitive::CAPSULE;
    } else if(type.compare("DONUT") == 0) {
        return mapit::msgs::Primitive::DONUT;
    } else if(type.compare("DISC") == 0) {
        return mapit::msgs::Primitive::DISC;
    } else if(type.compare("POINT") == 0) {
        return mapit::msgs::Primitive::POINT;
    } else if(type.compare("ARROW") == 0) {
        return mapit::msgs::Primitive::ARROW;
    } else if(type.compare("LINE") == 0) {
        return mapit::msgs::Primitive::LINE;
    } else if(type.compare("TEXT") == 0) {
        return mapit::msgs::Primitive::TEXT;
    } else if(type.compare("ICON") == 0) {
        return mapit::msgs::Primitive::ICON;
    } else {
        return mapit::msgs::Primitive::TEXT;
    }
}


upns::StatusCode operate_load_primitive(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();

    QJsonObject primitive = params["primitive"].toObject();

    std::string type = primitive["type"].toString().toStdString();

    std::shared_ptr<Entity> entity(new Entity);
    entity->set_type(PrimitiveEntitydata::TYPENAME());
    StatusCode s = env->getCheckout()->storeEntity(target, entity);
    if(!upnsIsOk(s))
    {
        log_error("Failed to create entity.");
    }

    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    if(abstractEntitydata == NULL)
    {
        return UPNS_STATUS_ERR_UNKNOWN;
    }
    std::shared_ptr<PrimitiveEntitydata> entityData = std::dynamic_pointer_cast<PrimitiveEntitydata>( abstractEntitydata );
    if(entityData == NULL)
    {
         // Because the Entity is stored (above) with the correct entity type, this should never happen!
        log_error("Primitive has wrong type. (This should never happen)");
        return UPNS_STATUS_ERR_UNKNOWN;
    }
    PrimitivePtr primitiveMsg(new mapit::msgs::Primitive);

    primitiveMsg->set_type(stringToType(type));
    entityData->setData(primitiveMsg);

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a primitive from JSON File", "fhac", OPERATOR_VERSION, PrimitiveEntitydata_TYPENAME, &operate_load_primitive)
