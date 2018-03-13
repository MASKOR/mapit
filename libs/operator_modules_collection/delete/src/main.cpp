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

#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/errorcodes.h>
#include <string>

#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

upns::StatusCode
deleteEntityOrTree(upns::CheckoutRaw* checkout, std::string entityName)
{
    std::shared_ptr<mapit::msgs::Entity> entity = checkout->getEntity(entityName);
    if (entity != nullptr) {
        upns::StatusCode s = checkout->deleteEntity(entityName);
        if ( ! upnsIsOk(s) ) {
            log_error("operator_delete: Error while deleting entity \"" + entityName + "\"");
            return s;
        }
    } else {
        std::shared_ptr<mapit::msgs::Tree> tree = checkout->getTree(entityName);
        if (tree != nullptr) {
            upns::StatusCode s = checkout->deleteTree(entityName);
            if ( ! upnsIsOk(s) ) {
                log_error("operator_delete: Error while deleting tree \"" + entityName + "\"");
                return s;
            }
        } else {
            log_warn("operator_delete: can't delete entity \"" + entityName + "\", does not exists");
//            return UPNS_STATUS_ERROR;
        }
    }

    return UPNS_STATUS_OK;
}

upns::StatusCode
operateDelete(upns::OperationEnvironment* env)
{
    /** structure:
     * {
     *  "target" : ["name_of_bag_1", ...]
     * }
     */
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    upns::CheckoutRaw* checkout = env->getCheckout();

    if (        params["target"].isString() ) {
        // just one entity/tree
        std::string entityName = params["target"].toString().toStdString();
        upns::StatusCode s = deleteEntityOrTree(checkout, entityName);
        if ( ! upnsIsOk(s) ) {
//            log_error("operator_delete: Error while deleting entity \"" + entityName + "\"");
            return s;
        }
    } else if ( params["target"].isArray() ) {
        // list of entity/tree
        QJsonArray jsonEntityNames( params["target"].toArray() );
        for (auto jsonEntityName : jsonEntityNames) {
            std::string entityName = jsonEntityName.toString().toStdString();
            upns::StatusCode s = deleteEntityOrTree(checkout, entityName);
            if ( ! upnsIsOk(s) ) {
    //            log_error("operator_delete: Error while deleting entity \"" + entityName + "\"");
                return s;
            }
        }
    } else {
        // unknown
        log_error("operator_delete: parameter \"target\" is either string nor array");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "delete entities", "fhac", OPERATOR_VERSION, "any", &operateDelete)
