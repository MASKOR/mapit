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

#include "entitydataloader.h"
#include <QJsonObject>
#include <QJsonArray>
#include <upns/versioning/repository.h>
#include <upns/versioning/checkout.h>
#include <upns/abstractentitydata.h>
#include <upns/versioning/repositoryfactorystandard.h>
#include <upns/logging.h>

#include <upns/layertypes/pointcloudlayer.h>
#if WITH_LAS
#include <upns/layertypes/lastype.h>
#endif // WITH_LAS
#include <upns/layertypes/openvdblayer.h>
#include <upns/layertypes/tflayer.h>
#include <upns/layertypes/pose_path.h>
#include <upns/layertypes/primitive.h>


QString typeToString(mapit::msgs::Primitive::PrimitiveType type)
{
    switch(type) {
    case mapit::msgs::Primitive::SPHERE: return "SPHERE";
    case mapit::msgs::Primitive::CUBE: return "CUBE";
    case mapit::msgs::Primitive::PLANE: return "PLANE";
    case mapit::msgs::Primitive::CYLINDER: return "CYLINDER";
    case mapit::msgs::Primitive::CONE: return "CONE";
    case mapit::msgs::Primitive::CAPSULE: return "CAPSULE";
    case mapit::msgs::Primitive::TORUS: return "TORUS";
    case mapit::msgs::Primitive::DISC: return "DISC";
    case mapit::msgs::Primitive::POINT: return "POINT";
    case mapit::msgs::Primitive::ARROW: return "ARROW";
    case mapit::msgs::Primitive::LINE: return "LINE";
    case mapit::msgs::Primitive::TEXT: return "TEXT";
    case mapit::msgs::Primitive::ICON: return "ICON";
    default: return "unknown";
    }
}

QJsonObject extractInfoFromEntitydata(std::shared_ptr<upns::AbstractEntitydata> entitydata)
{
    QJsonObject result;
    if(entitydata == nullptr) return result;
//#if WITH_PCL OR/AND OPENVDB
    if(strcmp(entitydata->type(), PointcloudEntitydata::TYPENAME()) == 0 || strcmp(entitydata->type(), FloatGridEntitydata::TYPENAME()) == 0)
    {
        std::shared_ptr<PointcloudEntitydata> entitydataPointcloud;
        std::shared_ptr<FloatGridEntitydata> entitydataOpenVDB;
        if(entitydataPointcloud = std::dynamic_pointer_cast<PointcloudEntitydata>( entitydata ))
        {
            upnsPointcloud2Ptr pc2 = entitydataPointcloud->getData();
            result["width"] = QJsonValue(static_cast<qint64>(pc2->width));
            result["length"] = QJsonValue(static_cast<qint64>(pc2->width));
            float xMin, yMin, zMin, xMax, yMax, zMax;
            entitydataPointcloud->getEntityBoundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
            QJsonObject minVec3;
            minVec3["x"] = QJsonValue( xMin );
            minVec3["y"] = QJsonValue( yMin );
            minVec3["z"] = QJsonValue( zMin );
            result["min"] = minVec3;
            QJsonObject maxVec3;
            maxVec3["x"] = QJsonValue( xMax );
            maxVec3["y"] = QJsonValue( yMax );
            maxVec3["z"] = QJsonValue( zMax );
            result["max"] = maxVec3;
            QJsonArray returnPointfieldList;
            for(std::vector<::pcl::PCLPointField>::iterator iter(pc2->fields.begin());
                iter != pc2->fields.end();
                iter++)
            {
                //TODO: this is only deleted, when entitydata is deleted via QObject parents.
                // Lifecycle is the same as m_entity! After m_entity changed, these pointfields must not be accessed anymore.
                QJsonObject field;
                field["name"] = QJsonValue( QString::fromStdString( iter->name ) );
                returnPointfieldList.append(field);
            }
            result["fields"] = returnPointfieldList;
        }
        else if(entitydataOpenVDB = std::dynamic_pointer_cast<FloatGridEntitydata>( entitydata ))
        {
            float xMin, yMin, zMin, xMax, yMax, zMax;
            entitydataOpenVDB->getEntityBoundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
            QJsonObject minVec3;
            minVec3["x"] = QJsonValue( xMin );
            minVec3["y"] = QJsonValue( yMin );
            minVec3["z"] = QJsonValue( zMin );
            result["min"] = minVec3;
            QJsonObject maxVec3;
            maxVec3["x"] = QJsonValue( xMax );
            maxVec3["y"] = QJsonValue( yMax );
            maxVec3["z"] = QJsonValue( zMax );
            result["max"] = maxVec3;
            //For now ovdb is visualized as a pointcloud, so
            // this emulates pointcloud visualization for float grids
            QJsonArray returnPointfieldList;
            QJsonObject jsX; jsX["name"] = "x";
            QJsonObject jsY; jsY["name"] = "y";
            QJsonObject jsZ; jsZ["name"] = "z";
            QJsonObject jsNX; jsNX["name"] = "normal_x";
            QJsonObject jsNY; jsNY["name"] = "normal_y";
            QJsonObject jsNZ; jsNZ["name"] = "normal_z";
            returnPointfieldList.append(jsX);
            returnPointfieldList.append(jsY);
            returnPointfieldList.append(jsZ);
            returnPointfieldList.append(jsNX);
            returnPointfieldList.append(jsNY);
            returnPointfieldList.append(jsNZ);
            result["fields"] = returnPointfieldList;
        }
    }
    else
//#endif
#if WITH_LAS
    if(strcmp(entitydata->type(), LASEntitydata::TYPENAME()) == 0)
    {
        std::shared_ptr<LASEntitydata> entityData = std::dynamic_pointer_cast<LASEntitydata>( entitydata );
        if(entityData != nullptr)
        {
            std::unique_ptr<LASEntitydataReader> rdr = entityData->getReader();
            qint64 w = static_cast<qint64>(rdr->GetHeader().GetPointRecordsCount());
            result["width"] = QJsonValue(w);
            result["length"] = QJsonValue(w);
        }
        if(entityData == nullptr)
        {
            QJsonObject minVec3;
            minVec3["x"] = QJsonValue(std::numeric_limits<float>::quiet_NaN());
            minVec3["y"] = QJsonValue(std::numeric_limits<float>::quiet_NaN());
            minVec3["z"] = QJsonValue(std::numeric_limits<float>::quiet_NaN());
            result["min"] = minVec3;
        }
        else
        {
            std::unique_ptr<LASEntitydataReader> rdr = entityData->getReader();
            QJsonObject minVec3;
            minVec3["x"] = QJsonValue(rdr->GetHeader().GetMinX());
            minVec3["y"] = QJsonValue(rdr->GetHeader().GetMinY());
            minVec3["z"] = QJsonValue(rdr->GetHeader().GetMinZ());
            result["min"] = minVec3;
        }
        if(entityData == nullptr)
        {
            QJsonObject maxVec3;
            maxVec3["x"] = QJsonValue(std::numeric_limits<float>::quiet_NaN());
            maxVec3["y"] = QJsonValue(std::numeric_limits<float>::quiet_NaN());
            maxVec3["z"] = QJsonValue(std::numeric_limits<float>::quiet_NaN());
            result["min"] = maxVec3;
        }
        else
        {
            std::unique_ptr<LASEntitydataReader> rdr = entityData->getReader();
            QJsonObject maxVec3;
            maxVec3["x"] = QJsonValue(rdr->GetHeader().GetMinX());
            maxVec3["y"] = QJsonValue(rdr->GetHeader().GetMinY());
            maxVec3["z"] = QJsonValue(rdr->GetHeader().GetMinZ());
            result["min"] = maxVec3;
        }
        //TODO: LAS has no "fields" nevertheless it has x, y, c and data properties. See Qt3DPontcloudRenderer.
        QJsonArray returnPointfieldList;
        QJsonObject jsX; jsX["name"] = "x";
        QJsonObject jsY; jsY["name"] = "y";
        QJsonObject jsZ; jsZ["name"] = "z";
        returnPointfieldList.append(jsX);
        returnPointfieldList.append(jsY);
        returnPointfieldList.append(jsZ);
        result["fields"] = returnPointfieldList;
    }
    else
#endif // WITH_LAS
    if(strcmp(entitydata->type(), TfEntitydata::TYPENAME()) == 0)
    {
        // has special qml wrapper
    }
    else if(strcmp(entitydata->type(), PosePathEntitydata::TYPENAME()) == 0)
    {
        std::shared_ptr<PosePathEntitydata> entityData = std::dynamic_pointer_cast<PosePathEntitydata>( entitydata );
        if(entityData != nullptr)
        {
            PosePathPtr pp = entityData->getData();
            qint64 w = static_cast<qint64>(pp->poses_size());
            result["width"] = QJsonValue(w);
            result["length"] = QJsonValue(w);
        }
    }
    else if(strcmp(entitydata->type(), PrimitiveEntitydata::TYPENAME()) == 0)
    {
        std::shared_ptr<PrimitiveEntitydata> primitiveEntitydata = std::dynamic_pointer_cast<PrimitiveEntitydata>( entitydata );
        if(primitiveEntitydata != nullptr)
        {
            PrimitivePtr prim = primitiveEntitydata->getData();
            result["type"] = typeToString(prim->type());
            result["text"] = QString::fromStdString(prim->text());
            float xMin, yMin, zMin, xMax, yMax, zMax;
            primitiveEntitydata->getEntityBoundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
            QJsonObject minVec3;
            minVec3["x"] = QJsonValue( xMin );
            minVec3["y"] = QJsonValue( yMin );
            minVec3["z"] = QJsonValue( zMin );
            result["min"] = minVec3;
            QJsonObject maxVec3;
            maxVec3["x"] = QJsonValue( xMax );
            maxVec3["y"] = QJsonValue( yMax );
            maxVec3["z"] = QJsonValue( zMax );
            result["max"] = maxVec3;
        }
    }
    else
    {
        log_info("Entitydataloader: unknown entitytype; can not obtain futher information");
    }
}

void EntitydataLoader::run()
{
    std::shared_ptr<upns::Checkout> co = m_checkout;
//    if(co == nullptr) {
//        std::unique_ptr<upns::Repository> repo( upns::RepositoryFactoryStandard::openRepositorySimple( m_repository.toStdString(), false) );
//        if(repo == nullptr) {
//            log_error("AsyncEntitydataLoader: could not load entitydata. Repository invalid");
//            return;
//        }
//        co = repo->getCheckout(m_checkoutname.toStdString());
//        if(co == nullptr) {
//            log_error("AsyncEntitydataLoader: could not load entitydata. Checkout not found");
//            return;
//        }
//    }
    if(co == nullptr) {
        log_error("AsyncEntitydataLoader: could not load entitydata. No Checkout specified");
        return;
    }
    std::shared_ptr<upns::AbstractEntitydata> ed = co->getEntitydataReadOnly(m_path.toStdString());
    if(ed == nullptr) {
        log_error("AsyncEntitydataLoader: could not load entitydata. Entitydata could not be read");
        return;
    }

    QJsonObject result = extractInfoFromEntitydata(ed);
    Q_EMIT entityInfoLoaded(result);
}

//EntitydataLoader::EntitydataLoader(QObject *parent, QString repository, QString checkoutname, QString path )
//    : QThread(parent)
//    , m_repository( repository )
//    , m_checkoutname( checkoutname )
//    , m_path( path )
//    , m_checkout( nullptr )
//{}

EntitydataLoader::EntitydataLoader(QObject *parent, std::shared_ptr<upns::Checkout> co, QString path)
    : QThread(parent)
    , m_repository( "" )
    , m_checkoutname( "" )
    , m_path( path )
    , m_checkout( co )
{

}

EntitydataLoader::~EntitydataLoader() {wait();}
