/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#include <mapit/operators/module.h>
#include <mapit/logging.h>
#include <mapit/layertypes/primitive.h>
#include <mapit/operators/versioning/checkoutraw.h>
#include <mapit/operators/operationenvironment.h>
#include <mapit/operators/versioning/checkoutraw.h>
#include <iostream>
#include <memory>
#include <mapit/errorcodes.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
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
    } else if(type.compare("TORUS") == 0) {
        return mapit::msgs::Primitive::TORUS;
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


mapit::StatusCode operate_load_primitive(mapit::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();

    //QJsonObject primitive = params["primitive"].toObject();

    std::string type = params["type"].toString().toUpper().toStdString();

    std::string frameId = params["frame_id"].toString().toStdString();

    if(target.empty())
    {
        log_warn("no target given");
        return MAPIT_STATUS_ERR_UNKNOWN;
    }
    if(type.empty())
    {
        log_warn("no type given");
        return MAPIT_STATUS_ERR_UNKNOWN;
    }

    std::shared_ptr<Entity> entity(new Entity);
    entity->set_type(PrimitiveEntitydata::TYPENAME());
    entity->set_frame_id(frameId);
    mapit::StatusCode s = env->getCheckout()->storeEntity(target, entity);
    if(!upnsIsOk(s))
    {
        log_error("Failed to create entity.");
    }

    std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    if(abstractEntitydata == NULL)
    {
        return MAPIT_STATUS_ERR_UNKNOWN;
    }
    std::shared_ptr<PrimitiveEntitydata> entityData = std::dynamic_pointer_cast<PrimitiveEntitydata>( abstractEntitydata );
    if(entityData == NULL)
    {
         // Because the Entity is stored (above) with the correct entity type, this should never happen!
        log_error("Primitive has wrong type. (This should never happen)");
        return MAPIT_STATUS_ERR_UNKNOWN;
    }
    PrimitivePtr primitiveMsg(new mapit::msgs::Primitive);

    primitiveMsg->set_type(stringToType(type));
    entityData->setData(primitiveMsg);

    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME, "Loads a primitive from JSON File", "fhac", OPERATOR_VERSION, PrimitiveEntitydata_TYPENAME, &operate_load_primitive)
