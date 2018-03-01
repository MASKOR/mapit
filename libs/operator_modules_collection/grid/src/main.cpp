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

#include <upns/errorcodes.h>
#include <upns/operators/module.h>
#include <upns/operators/operationenvironment.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <upns/logging.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>
#include <memory>
#include <pcl/octree/octree.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>

template <typename PointType>
pcl::PCLPointCloud2::Ptr extractIndices(pcl::PointCloud<PointType> &cloudIn, std::vector<int> &indices)
{
    pcl::PCLPointCloud2::Ptr cloudOut(new pcl::PCLPointCloud2);
    pcl::ExtractIndices< PointType > extractIndices;
    extractIndices.setInputCloud(cloudIn);
    extractIndices.setIndices(indices);
    extractIndices.filter(cloudOut);
}

void notDeleter(pcl::PCLPointCloud2* d) {}

upns::StatusCode operate_grid(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    double leafSize = params["leafsize"].toDouble();
    int cells = params["cells"].toInt();
    if(leafSize == 0.0)
    {
        if(cells == 0)
        {
            leafSize = 0.01f;
            cells = 0; //TODO: calc
        }
        else
        {
            //TODO
            //leafSize = static_cast<double>(maxSize)/static_cast<double>(cells);
        }

    }

    std::string target = params["target"].toString().toStdString();

    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
    if(entityData == nullptr)
    {
        log_error("Wrong type (not a pointcloud)");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    upnsPointcloud2Ptr pc2 = entityData->getData();

    //TODO: Helper Library: Query for entities in frustum. Query for level of detail.
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(*pc2, *pc);
    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree(leafSize);

    // add point data to octree
    octree.setInputCloud( pc );
    octree.addPointsFromInputCloud();

    std::string prefix(target);
    if(target.at(target.length()-1) != '/')
    {
        prefix += '/';
    }


    // leaf node iterator
    for(pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ>::LeafNodeIterator it(octree.leaf_begin()); it != octree.leaf_end() ; ++it)
    {
        //Iteratively explore only the leaf nodes..
        std::vector<int> indexVector;
        pcl::octree::OctreeContainerPointIndices& container = it.getLeafContainer();
        container.getPointIndices (indexVector);

//        pcl::octree::OctreeKey &key = it.getCurrentOctreeKey();
//        std::string postfix = std::to_string(key.x) + "_" + std::to_string(key.y) + "_"  + std::to_string(key.z);

//        pcl::ExtractIndices< pcl::PCLPointCloud2 > extractIndices;
//        extractIndices.setInputCloud(pc2);
//        extractIndices.setIndices(indexVector);
//        pcl::PCLPointCloud2 p2;
//        extractIndices.filter(p2);

//        std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( prefix + postfix );
//        std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
//        if(entityData == nullptr)
//        {
//            log_error("Wrong type");
//            return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
//        }
//        entityData->setData(p2);
    }

//    OperationDescription out;
//    out.set_operatorname(OPERATOR_NAME);
//    out.set_operatorversion(OPERATOR_VERSION);
//    env->setOutputDescription( out.SerializeAsString() );
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "slice a pointcloud into multiple pointclouds using a grid", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, &operate_grid)
