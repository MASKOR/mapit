/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <iostream>
#include <memory>
#include <mapit/errorcodes.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <QFile>

using namespace mapit::msgs;

mapit::StatusCode operate_edit_entity(mapit::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();

    bool frameIdExists = params["frame_id"].isString() && ! params["frame_id"].toString().isEmpty();
    std::string frameId;
    if(frameIdExists)
    {
        frameId = params["frame_id"].toString().toStdString();
    }
    bool stampExists = params["stamp"].isObject()
                    && (
                           params["stamp"].toObject()["sec"].toInt() != 0
                        || params["stamp"].toObject()["nsec"].toInt() != 0
                       );
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
        return MAPIT_STATUS_OK;
    }

    std::shared_ptr<mapit::msgs::Entity> entity = env->getWorkspace()->getEntity(target);
    if(entity == nullptr)
    {
        log_error("Entity not found.");
        return MAPIT_STATUS_ERR_DB_NOT_FOUND;
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
    env->getWorkspace()->storeEntity(target, entity);

    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME, "Loads a Path from JSON File", "fhac", OPERATOR_VERSION, "*", true, &operate_edit_entity)
