/*******************************************************************************
 *
 * Copyright      2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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
#include <mapit/errorcodes.h>
#include <string>

#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

mapit::StatusCode
deleteEntityOrTree(mapit::operators::WorkspaceWritable* workspace, std::string entityName)
{
    std::shared_ptr<mapit::msgs::Entity> entity = workspace->getEntity(entityName);
    if (entity != nullptr) {
        mapit::StatusCode s = workspace->deleteEntity(entityName);
        if ( ! mapitIsOk(s) ) {
            log_error("operator_delete: Error while deleting entity \"" + entityName + "\"");
            return s;
        }
    } else {
        std::shared_ptr<mapit::msgs::Tree> tree = workspace->getTree(entityName);
        if (tree != nullptr) {
            mapit::StatusCode s = workspace->deleteTree(entityName);
            if ( ! mapitIsOk(s) ) {
                log_error("operator_delete: Error while deleting tree \"" + entityName + "\"");
                return s;
            }
        } else {
            log_warn("operator_delete: can't delete entity \"" + entityName + "\", does not exists");
//            return MAPIT_STATUS_ERROR;
        }
    }

    return MAPIT_STATUS_OK;
}

mapit::StatusCode
operateDelete(mapit::OperationEnvironment* env)
{
    /** structure:
     * {
     *  "target" : ["name_of_bag_1", ...]
     * }
     */
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    mapit::operators::WorkspaceWritable* workspace = env->getWorkspace();

    if (        params["target"].isString() ) {
        // just one entity/tree
        std::string entityName = params["target"].toString().toStdString();
        mapit::StatusCode s = deleteEntityOrTree(workspace, entityName);
        if ( ! mapitIsOk(s) ) {
//            log_error("operator_delete: Error while deleting entity \"" + entityName + "\"");
            return s;
        }
    } else if ( params["target"].isArray() ) {
        // list of entity/tree
        QJsonArray jsonEntityNames( params["target"].toArray() );
        for (auto jsonEntityName : jsonEntityNames) {
            std::string entityName = jsonEntityName.toString().toStdString();
            mapit::StatusCode s = deleteEntityOrTree(workspace, entityName);
            if ( ! mapitIsOk(s) ) {
    //            log_error("operator_delete: Error while deleting entity \"" + entityName + "\"");
                return s;
            }
        }
    } else {
        // unknown
        log_error("operator_delete: parameter \"target\" is either string nor array");
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }

    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME, "delete entities", "fhac", OPERATOR_VERSION, "any", &operateDelete)
