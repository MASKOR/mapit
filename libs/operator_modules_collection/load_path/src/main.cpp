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

upns::StatusCode operate_load_posepath(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();


    std::string filename = params["filename"].toString().toStdString();
    if(filename.empty())
    {
        log_error("parameter \"filename\" missing");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    std::shared_ptr<Entity> entity(new Entity);
    entity->set_type(PosePathEntitydata::TYPENAME());
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
    std::shared_ptr<PosePathEntitydata> entityData = std::dynamic_pointer_cast<PosePathEntitydata>( abstractEntitydata );
    if(entityData == NULL)
    {
         // Because the Entity is stored (above) with the correct entity type, this should never happen!
        log_error("Asset has wrong type. (This should never happen)");
        return UPNS_STATUS_ERR_UNKNOWN;
    }
    PosePathPtr posePath(new mapit::msgs::PosePath);

    QString jsonString;
    QFile file(QString::fromStdString(filename));
    if(!file.exists())
    {
        log_error("No such file");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    if(!file.isOpen())
    {
        log_error("Could not open file");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    if(!file.isReadable())
    {
        log_error("File not readable");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    jsonString = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonString.toUtf8(), &parseError);
    if(parseError.error != QJsonParseError::NoError)
    {
        log_error("Could not parse JSON: " + parseError.errorString().toStdString());
        return UPNS_STATUS_INVALID_DATA;
    }
    QJsonObject json(jsonDoc.object());
    if(json["points"].isArray())
    {
        QJsonArray points = json["points"].toArray();
        for(QJsonArray::const_iterator iter=points.constBegin() ; iter != points.constEnd() ; ++iter)
        {
            mapit::msgs::Pose *pose = posePath->add_poses();
            if(iter->isObject())
            {
                QJsonObject pt = iter->toObject();
                QJsonValue val = pt["x"];
                if(val.isDouble())
                {
                    pose->set_x( val.toDouble() );
                }
                else
                    log_error("point without x");
                val = pt["y"];
                if(val.isDouble())
                {
                    pose->set_y( val.toDouble() );
                }
                else
                    log_error("point without y");
                val = pt["z"];
                if(val.isDouble())
                {
                    pose->set_z( val.toDouble() );
                }
                else
                    log_error("point without z");
            }
            else
            {
                log_error("wrong json format. Can not interpret non object in \"points\" array");
            }
        }
    }
    else
    {
        log_error("wrong json format: there must be a \"points\" member of type Array");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    entityData->setData(posePath);

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Path from JSON File", "fhac", OPERATOR_VERSION, PosePathEntitydata_TYPENAME, &operate_load_posepath)
