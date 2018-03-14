/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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
#include <pcl/filters/voxel_grid.h>
#include <memory>
#include <mapit/errorcodes.h>
#include <mapit/operators/versioning/checkoutraw.h>
#include <mapit/depthfirstsearch.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

mapit::StatusCode executeVoxelgrid(mapit::OperationEnvironment* env, const std::string& target, const double& leafSize)
{
    std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
    if(entityData == nullptr)
    {
        log_error("Wrong type");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    upnsPointcloud2Ptr pc2 = entityData->getData();

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    pcl::PCLPointCloud2ConstPtr stdPc2( pc2.get(), [](pcl::PCLPointCloud2*){});
    sor.setInputCloud(stdPc2);
    sor.setLeafSize (leafSize, leafSize, leafSize);

    upnsPointcloud2Ptr cloud_filtered(new pcl::PCLPointCloud2 ());
    sor.filter (*cloud_filtered);
    std::stringstream strstr;
    strstr << "new pointcloudsize " << cloud_filtered->width << "(leafsize: " << leafSize << ")";
    log_info( strstr.str() );

    entityData->setData(cloud_filtered);

    return MAPIT_STATUS_OK;
}

mapit::StatusCode operate_vxg(mapit::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    log_info( "Voxelgrid params:" + env->getParameters() );
    QJsonObject params(paramsDoc.object());
    double leafSize = params["leafsize"].toDouble();

    if(leafSize == 0.0)
    {
        log_info( "Leafsize was 0, using 0.01" );
        leafSize = 0.01f;
    }

    std::string target = params["target"].toString().toStdString();

    if ( env->getCheckout()->getEntity(target) ) {
        // execute on entity
        return executeVoxelgrid(env, target, leafSize);
    } else if ( env->getCheckout()->getTree(target) ) {
        // execute on tree
        mapit::StatusCode status = MAPIT_STATUS_OK;
        env->getCheckout()->depthFirstSearch(
                      target
                    , depthFirstSearchAll(mapit::msgs::Tree)
                    , depthFirstSearchAll(mapit::msgs::Tree)
                    , [&](std::shared_ptr<mapit::msgs::Entity> obj, const ObjectReference& ref, const mapit::Path &path)
                        {
                            status = executeVoxelgrid(env, path, leafSize);
                            if ( ! upnsIsOk(status) ) {
                                return false;
                            }
                            return true;
                        }
                    , depthFirstSearchAll(mapit::msgs::Entity)
                    );
        return status;
    } else {
        log_error("operator voxelgrid: target is neither a tree nor entity");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
}

MAPIT_MODULE(OPERATOR_NAME, "use pcl voxelgrid filter on a pointcloud", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, &operate_vxg)
