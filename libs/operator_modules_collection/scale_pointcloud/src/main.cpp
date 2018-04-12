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
#include <mapit/depthfirstsearch.h>
#include <string>

#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

#include <mapit/layertypes/pointcloudlayer.h>

double factorX_ = 0.0;
double factorY_ = 0.0;
double factorZ_ = 0.0;

void scalePoint(std::vector<pcl::uint8_t>& data, const size_t& offset, const ::pcl::PCLPointField& type, const double& factor)
{
    switch (type.datatype) {
        case ::pcl::PCLPointField::FLOAT32: {
            float* p = reinterpret_cast<float*>( &(data[offset + type.offset]) );
            (*p) *= factor;
        }
        break;
        case ::pcl::PCLPointField::FLOAT64: {
            double* p = reinterpret_cast<double*>( &(data[offset + type.offset]) );
            (*p) *= factor;
        }
        break;
        default:
            log_error("scale_pointcloud: pointtype is not implemented");
        break;
    }
}

mapit::StatusCode scalePointcloud(mapit::operators::WorkspaceWritable* workspace, std::string path, std::shared_ptr<mapit::msgs::Entity> entity)
{
    std::shared_ptr<mapit::AbstractEntitydata> edA = workspace->getEntitydataForReadWrite(path);
    if ( 0 != std::strcmp(edA->type(), PointcloudEntitydata::TYPENAME()) ) {
        log_error("scale_pointcloud: entity \"" + path
                + "\" is of type \"" + edA->type()
                + "\", but type \"" + PointcloudEntitydata::TYPENAME() + "\" is needed for this operator");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    std::shared_ptr<PointcloudEntitydata> ed = std::static_pointer_cast<PointcloudEntitydata>(edA);
    std::shared_ptr<pcl::PCLPointCloud2> edPointcloud = ed->getData();

    uint32_t pointSize = edPointcloud->point_step;
    uint32_t offsetX = -1;
    uint32_t offsetY = -1;
    uint32_t offsetZ = -1;
    ::pcl::PCLPointField typeX;
    ::pcl::PCLPointField typeY;
    ::pcl::PCLPointField typeZ;

    for (::pcl::PCLPointField field : edPointcloud->fields) {
        if (        0 == field.name.compare("x") ) {
            offsetX = field.offset;
            typeX = field;
        } else if ( 0 == field.name.compare("y") ) {
            offsetY = field.offset;
            typeY = field;
        } else if ( 0 == field.name.compare("z") ) {
            offsetZ = field.offset;
            typeZ = field;
        }
    }
    if (offsetX == -1 || offsetY == -1 || offsetZ == -1) {
        log_error("scale_pointcloud: can not extract offset of x, y, or z of pointcloud \"" + path + "\"");
        return MAPIT_STATUS_ERROR;
    }
    for (  int pointBegin = 0
         ; pointBegin < edPointcloud->data.size()
         ; pointBegin += pointSize) {
        scalePoint(edPointcloud->data, pointBegin, typeX, factorX_);
        scalePoint(edPointcloud->data, pointBegin, typeY, factorY_);
        scalePoint(edPointcloud->data, pointBegin, typeZ, factorZ_);
    }

    ed->setData(edPointcloud);

    return MAPIT_STATUS_OK;
}

mapit::StatusCode operateScalePointclouds(mapit::OperationEnvironment* env)
{
    /** structure:
     * {
     *  <string>"target" : ...,
     *  <float>"factor-x" : ...,
     *  <float>"factor-y" : ...,
     *  <float>"factor-z" : ...
     * }
     */
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    mapit::operators::WorkspaceWritable* workspace = env->getWorkspace();

    if ( ! params.contains("target") || ! params["target"].isString() ) {
        log_error("scale_pointcloud: parameter \"target\" is not set or not a string");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    std::string target = params["target"].toString().toStdString();
    if ( ! params.contains("factor-x") || ! params["factor-x"].isDouble() ) {
        log_error("scale_pointcloud: parameter \"factor-x\" is not set or not a float");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    factorX_ = params["factor-x"].toDouble();
    if ( ! params.contains("factor-y") || ! params["factor-y"].isDouble() ) {
        log_error("scale_pointcloud: parameter \"factor-y\" is not set or not a float");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    factorY_ = params["factor-y"].toDouble();
    if ( ! params.contains("factor-z") || ! params["factor-z"].isDouble() ) {
        log_error("scale_pointcloud: parameter \"factor-z\" is not set or not a float");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    factorZ_ = params["factor-z"].toDouble();

    log_info("scale_pointcloud: executing on \"" + target + "\" with factor ("
             + std::to_string(factorX_) + ", " + std::to_string(factorY_) + ", " + std::to_string(factorZ_) + ")");

    std::shared_ptr<mapit::msgs::Entity> entity = workspace->getEntity(target);
    if (entity != nullptr) {
        return scalePointcloud(workspace, target, entity);
    } else {
        std::shared_ptr<mapit::msgs::Tree> tree = workspace->getTree(target);
        if (tree != nullptr) {
            mapit::StatusCode statusSearch = MAPIT_STATUS_OK;
            workspace->depthFirstSearch(
                        target
                        , depthFirstSearchWorkspaceAll(mapit::msgs::Tree)
                        , depthFirstSearchWorkspaceAll(mapit::msgs::Tree)
                        , [&](std::shared_ptr<mapit::msgs::Entity> obj, const ObjectReference& ref, const mapit::Path &path)
                          {
                              mapit::StatusCode s = scalePointcloud(workspace, path, obj);
                              if ( mapitIsOk(s) ) {
                                  return true;
                              } else {
                                  statusSearch = s;
                                  return false;
                              }
                          }
                        , depthFirstSearchWorkspaceAll(mapit::msgs::Entity)
                        );
        } else {
            log_error("scale_pointcloud: \"target\" is neither a entity nor a tree");
            return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
        }
    }

    return MAPIT_STATUS_ERROR; // this shouldn't be reached
}

MAPIT_MODULE(OPERATOR_NAME, "scale pointclouds by a factor in x, y and z", "fhac", OPERATOR_VERSION, "any", true, &operateScalePointclouds)
