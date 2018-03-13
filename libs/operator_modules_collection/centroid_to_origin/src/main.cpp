/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include <upns/logging.h>
#include <upns/operators/module.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <upns/layertypes/tflayer.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <iostream>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <memory>
#include <upns/errorcodes.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>

using namespace mapit::msgs;

template <typename PointT>
StatusCode operateConcreteType(upnsPointcloud2Ptr cloud, std::shared_ptr<PointcloudEntitydata> entityData, upns::OperationEnvironment* env);

StatusCode operate(upnsPointcloud2Ptr cloud, std::shared_ptr<PointcloudEntitydata> entityData, upns::OperationEnvironment* env)
{
    bool hasX = false;
    bool hasY = false;
    bool hasZ = false;
    bool hasNormalX = false;
    bool hasNormalY = false;
    bool hasNormalZ = false;
    bool hasCurvature = false;
    bool hasRGB = false;
    bool hasIntensity = false;
    for(std::vector< ::pcl::PCLPointField>::const_iterator iter(cloud->fields.cbegin()); iter != cloud->fields.cend(); ++iter)
    {
        hasX |= iter->name.compare("x") == 0;
        hasY |= iter->name.compare("y") == 0;
        hasZ |= iter->name.compare("z") == 0;
        hasNormalX |= iter->name.compare("normal_x") == 0;
        hasNormalY |= iter->name.compare("normal_y") == 0;
        hasNormalZ |= iter->name.compare("normal_z") == 0;
        hasRGB |= iter->name.compare("rgb") == 0;
        hasIntensity |= iter->name.compare("intensity") == 0;
    }
    // combined
    bool hasXYZ = hasX == true
               && hasY == true
               && hasZ == true;
    bool hasNormal = hasX == true
                  && hasY == true
                  && hasZ == true;
    if(  hasXYZ == true
      && hasNormal == true
      && hasRGB == true
      && hasIntensity == true)
    {
        //intensity missing
        return operateConcreteType<pcl::PointXYZRGBNormal>(cloud, entityData, env);
    }
    else if(  hasXYZ == true
           && hasNormal == true
           && hasRGB == true
           && hasIntensity == false)
    {
        return operateConcreteType<pcl::PointXYZRGBNormal>(cloud, entityData, env);
    }
    else if(  hasXYZ == true
           && hasNormal == true
           && hasRGB == false
           && hasIntensity == true)
    {
        return operateConcreteType<pcl::PointXYZINormal>(cloud, entityData, env);
    }
    else if(  hasXYZ == true
           && hasNormal == true
           && hasRGB == false
           && hasIntensity == false)
    {
        //Intensity must be there
        return operateConcreteType<pcl::PointXYZINormal>(cloud, entityData, env);
    }
    else if(  hasXYZ == true
           && hasNormal == false
           && hasRGB == true
           && hasIntensity == true)
    {
        //Intensity is A
        return operateConcreteType<pcl::PointXYZRGBA>(cloud, entityData, env);
    }
    else if(  hasXYZ == true
           && hasNormal == false
           && hasRGB == true
           && hasIntensity == false)
    {
        return operateConcreteType<pcl::PointXYZRGB>(cloud, entityData, env);
    }
    else if(  hasXYZ == true
           && hasNormal == false
           && hasRGB == false
           && hasIntensity == true)
    {
        return operateConcreteType<pcl::PointXYZI>(cloud, entityData, env);
    }
    else if(  hasXYZ == true
           && hasNormal == false
           && hasRGB == false
           && hasIntensity == false)
    {
        return operateConcreteType<pcl::PointXYZ>(cloud, entityData, env);
    }
    else if(  hasXYZ == false
           && hasNormal == true
           && hasRGB == true
           && hasIntensity == true)
    {
        // no RGB, no I
        log_error("Pointcloud2 incomatible (only normal).");
        //return operateConcreteType<pcl::PointNormal>(cloud, entityData, env);
    }
    else if(  hasXYZ == false
           && hasNormal == true
           && hasRGB == true
           && hasIntensity == false)
    {
        // no RGB
        log_error("Pointcloud2 incomatible (only normal).");
        //return operateConcreteType<pcl::PointNormal>(cloud, entityData, env);
    }
    else if(  hasXYZ == false
           && hasNormal == true
           && hasRGB == false
           && hasIntensity == true)
    {
        // no I
        log_error("Pointcloud2 incomatible (only normal).");
        //return operateConcreteType<pcl::PointNormal>(cloud, entityData, env);
    }
    else if(  hasXYZ == false
           && hasNormal == true
           && hasRGB == false
           && hasIntensity == false)
    {
        log_error("Pointcloud2 incomatible (only normal).");
        //return operateConcreteType<pcl::PointNormal>(cloud, entityData, env);
    }
    else if(  hasXYZ == false
           && hasNormal == false
           && hasRGB == true
           && hasIntensity == true)
    {
        // XYZ must be there, I is A
        return operateConcreteType<pcl::PointXYZRGBA>(cloud, entityData, env);
    }
    else if(  hasXYZ == false
           && hasNormal == false
           && hasRGB == true
           && hasIntensity == false)
    {
        // XYZ must be there
        return operateConcreteType<pcl::PointXYZRGB>(cloud, entityData, env);
    }
    else if(  hasXYZ == false
           && hasNormal == false
           && hasRGB == false
           && hasIntensity == true)
    {
        // XYZ must be there
        return operateConcreteType<pcl::PointXYZI>(cloud, entityData, env);
    }
    else if(  hasXYZ == false
           && hasNormal == false
           && hasRGB == false
           && hasIntensity == false)
    {
        log_error("Pointcloud2 has no fields.");
    }
    return UPNS_STATUS_ERROR;
}

//StatusCode demean(upnsPointcloud2Ptr cloud, Eigen::Vector4f ctr, pcl::PointCloud<pcl::PointXYZ> *pcCtr)
//{
//    pcl::demeanPointCloud(*cloud, ctr, *pcCtr);
//}

//template <typename PointT>
//StatusCode demean(upnsPointcloud2Ptr cloud, Eigen::Vector4f ctr, pcl::PointCloud<PointT> *pcCtr)
//{

//}

template <typename PointT>
StatusCode operateConcreteType(upnsPointcloud2Ptr cloud, std::shared_ptr<PointcloudEntitydata> entityData, upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    bool useAxisAlignedBoundingBox = params["useAABB"].toBool();
    bool genTf = params["genTf"].toBool();

    pcl::PointCloud<pcl::PointXYZ> pc; // TODO: make generic
    pcl::PointCloud<PointT> pcAll; // TODO: make generic
    pcl::fromPCLPointCloud2(*cloud, pc);
    pcl::fromPCLPointCloud2(*cloud, pcAll);

    Eigen::Vector4f ctr;
    if(useAxisAlignedBoundingBox)
    {
        pcl::PointXYZ min;
        pcl::PointXYZ max;
        pcl::getMinMax3D(pc, min, max);
        log_info("Centroid to origin: Minimum was ("
                 + std::to_string(min.x) + ", "
                 + std::to_string(min.y) + ", "
                 + std::to_string(min.z) + "), Maximum: ("
                 + std::to_string(max.x) + ", "
                 + std::to_string(max.y) + ", "
                 + std::to_string(max.z) + ")");
        ctr = Eigen::Vector4f((min.x+max.x)/2.0f, (min.y+max.y)/2.0f, (min.z+max.z)/2.0f, 1.0f);
    }
    else
    {
        pcl::compute3DCentroid(pc, ctr);
        log_info("Centroid to origin: Centroid was ("
                 + std::to_string(ctr[0]) + ", "
                 + std::to_string(ctr[1]) + ", "
                 + std::to_string(ctr[2]) + ")");
    }
//    if(!genTf)
//    {
        pcl::PointCloud<PointT> pcCtr;
        pcl::demeanPointCloud(pcAll, ctr, pcCtr);
        //demean(pc, ctr, &pcCtr);

        upnsPointcloud2Ptr pc2Out(new pcl::PCLPointCloud2());
        pcl::toPCLPointCloud2( pcCtr, *pc2Out);

        entityData->setData(pc2Out);
        PointT min;
        PointT max;
        pcl::getMinMax3D(pcCtr, min, max);
        log_info("Centroid to origin: New minimum is ("
                 + std::to_string(min.x) + ", "
                 + std::to_string(min.y) + ", "
                 + std::to_string(min.z) + "), Maximum: ("
                 + std::to_string(max.x) + ", "
                 + std::to_string(max.y) + ", "
                 + std::to_string(max.z) + ")");
//    }
//    else
//    {
//        std::string tfEntityName = params["tfTarget"].toString().toStdString();
//        // Get Target
//        std::shared_ptr<mapit::msgs::Entity> tfEntity = env->getCheckout()->getEntity(tfEntityName);
//        if(tfEntity == NULL)
//        {
//            // If target could not be received, create new entity
//            // There must be an entity in order to write entity data in the next step
//            tfEntity = std::shared_ptr<mapit::msgs::Entity>(new mapit::msgs::Entity);
//            tfEntity->set_type(TfEntitydata::TYPENAME());
//            StatusCode s = env->getCheckout()->storeEntity(tfEntityName, tfEntity);
//            if(!upnsIsOk(s))
//            {
//                log_error("Failed to create transform entity.");
//                return UPNS_STATUS_ERR_UNKNOWN;
//            }
//        }
//        std::shared_ptr<AbstractEntitydata> abstractEntitydataTf = env->getCheckout()->getEntitydataForReadWrite( tfEntityName );
//        if(abstractEntitydataTf == NULL)
//        {
//            return UPNS_STATUS_ERR_UNKNOWN;
//        }
//        std::shared_ptr<TfEntitydata> entityDataTf = std::dynamic_pointer_cast<TfEntitydata>( abstractEntitydataTf );
//        if(entityDataTf == NULL)
//        {
//            log_error("Tf Transform has wrong type.");
//            return UPNS_STATUS_ERR_UNKNOWN;
//        }
//        upns::tf::TransformPtr tf = upns::tf::TransformPtr(new upns::tf::Transform);
//        tf->translation = Eigen::Translation3f(-ctr[0], -ctr[1], -ctr[2]);
//        entityDataTf->setData(tf);
//    }
    return UPNS_STATUS_OK;
}

upns::StatusCode operate_ctr(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();

    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
    if(entityData == nullptr)
    {
        log_error("Wrong type");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    upnsPointcloud2Ptr pc2 = entityData->getData();

    return operate(pc2, entityData, env);
}

UPNS_MODULE(OPERATOR_NAME, "use point of mass as origin of the pointcloud", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, &operate_ctr)
