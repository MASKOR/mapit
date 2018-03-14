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

#include <mapit/operators/module.h>
#include <mapit/logging.h>
#include <mapit/layertypes/pointcloudlayer.h>
#include <mapit/operators/versioning/checkoutraw.h>
#include <mapit/operators/operationenvironment.h>
#include <iostream>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>
#include <pcl/point_types.h>
#include <pcl/surface/impl/mls.hpp> //error when loading libpcl_surface.so. So just include implementation here
#include <memory>
#include <mapit/errorcodes.h>
#include <mapit/operators/versioning/checkoutraw.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>


template <typename PointT>
mapit::StatusCode operateConcreteType(upnsPointcloud2Ptr cloud, std::shared_ptr<PointcloudEntitydata> entityData, mapit::OperationEnvironment* env);

mapit::StatusCode operate(upnsPointcloud2Ptr cloud, std::shared_ptr<PointcloudEntitydata> entityData, mapit::OperationEnvironment* env)
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
    return MAPIT_STATUS_ERROR;
}

template <typename PointT>
mapit::StatusCode operateConcreteType(upnsPointcloud2Ptr cloud, std::shared_ptr<PointcloudEntitydata> entityData, mapit::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    log_info( "moving least squares params:" + env->getParameters() );
    QJsonObject params(paramsDoc.object());
    double radius = params["radius"].toDouble();

    if(radius == 0.0)
    {
        log_info( "radius was 0, using 0.01" );
        radius = 0.01f;
    }
    int polynomialOrder = params["polynomial"].toInt();
    bool computeNormals = params["computeNormals"].toBool();

    typename pcl::PointCloud<PointT>::Ptr pcAll(new pcl::PointCloud<PointT>());
    pcl::fromPCLPointCloud2(*cloud, *pcAll);

    // Create a KD-Tree
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<PointT> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointT, PointT> mls;

    mls.setComputeNormals (computeNormals);

    // Set parameters
    mls.setInputCloud (pcAll);
    mls.setPolynomialOrder( polynomialOrder );
    mls.setSearchMethod (tree);
    mls.setSearchRadius (radius);

    // Reconstruct
    mls.process (mls_points);

    std::stringstream strstr;
    strstr << "new pointcloudsize " << mls_points.width << "(radius: " << radius << ")";
    log_info( strstr.str() );

    upnsPointcloud2Ptr cloudOut(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(mls_points, *cloudOut);
    entityData->setData(cloudOut);

    return MAPIT_STATUS_OK;
}


mapit::StatusCode operate_mls(mapit::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    log_info( "moving least squares params:" + env->getParameters() );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();

    std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
    if(entityData == nullptr)
    {
        log_error("Wrong type");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    upnsPointcloud2Ptr pc2 = entityData->getData();

    return operate(pc2, entityData, env);
}

MAPIT_MODULE(OPERATOR_NAME, "resample data using moving least squares. Smooths normals or color", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, &operate_mls)
