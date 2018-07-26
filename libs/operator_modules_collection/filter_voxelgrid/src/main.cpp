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
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>
#include <iostream>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <memory>
#include <mapit/errorcodes.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/depthfirstsearch.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <string>

mapit::StatusCode executeVoxelgrid(mapit::OperationEnvironment* env, const std::string& target, const double& octreeLeafSize, const double& leafSize)
{
    std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydata = env->getWorkspace()->getEntitydataForReadWrite( target );
    std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
    if(entityData == nullptr)
    {
        log_error("Wrong type");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    mapit::entitytypes::Pointcloud2Ptr pc2 = entityData->getData();
    mapit::entitytypes::Pointcloud2Ptr cloud_filtered = std::make_shared<pcl::PCLPointCloud2>();

    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud<Point> Cloud;

    boost::shared_ptr<Cloud> pc_octree_in = boost::make_shared<Cloud>();
    pcl::fromPCLPointCloud2(*pc2, *pc_octree_in);

    double bounding_box_size = int( octreeLeafSize / leafSize + 1) * leafSize;

    // use octree to apply this filter on sub-clouds (one cloud is too big)
    pcl::octree::OctreePointCloudPointVector<Point> octree_for_bounding_box_of_sub_clouds(bounding_box_size);
    octree_for_bounding_box_of_sub_clouds.setInputCloud( pc_octree_in );
    octree_for_bounding_box_of_sub_clouds.addPointsFromInputCloud();

    // for each sub-cloud
    size_t count_total = octree_for_bounding_box_of_sub_clouds.getLeafCount();
    log_info("Voxelgrid: process " + std::to_string(count_total) + " leafs of octree");
    size_t count = 1;
    for ( pcl::octree::OctreePointCloud<Point>::LeafNodeIterator it = octree_for_bounding_box_of_sub_clouds.leaf_begin();
          it != octree_for_bounding_box_of_sub_clouds.leaf_end();
          it++ ) {
      if (count % 10 == 0) {
          log_info("Voxelgrid: in leaf " + std::to_string(count) + "/" + std::to_string(count_total));
      }
      ++count;
      std::vector<int> indices_octree_leaf;
      it.getLeafContainer().getPointIndices(indices_octree_leaf);

      boost::shared_ptr<pcl::PCLPointCloud2> pc_octree_leaf = boost::make_shared<pcl::PCLPointCloud2>();
      pcl::PCLPointCloud2 pc_voxelgrid_cloud;
      pcl::copyPointCloud(*pc2, indices_octree_leaf, *pc_octree_leaf);

      // apply VoxelGrid filter on this sub-cloud
      pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
      vg.setLeafSize(leafSize, leafSize, leafSize);
      vg.setInputCloud(pc_octree_leaf);
      vg.filter(pc_voxelgrid_cloud);

      // add to final cloud
      std::shared_ptr<pcl::PCLPointCloud2> cloud_filtered_new = std::make_shared<pcl::PCLPointCloud2>();
      pcl::concatenatePointCloud(*cloud_filtered, pc_voxelgrid_cloud, *cloud_filtered_new);
      cloud_filtered = cloud_filtered_new;
    }

    log_info( "new pointcloudsize " << cloud_filtered->width << "(leafsize: " << leafSize << ")" );

    entityData->setData(cloud_filtered);

    return MAPIT_STATUS_OK;
}

mapit::StatusCode operate_vxg(mapit::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    log_info( "Voxelgrid params: " + env->getParameters() );
    QJsonObject params(paramsDoc.object());
    double leafSize = params["leafsize"].toDouble();
    double octreeLeafSize;
    if ( params["octree-leafsize"].isDouble() ) {
        octreeLeafSize = params["octree-leafsize"].toDouble();
    } else {
        octreeLeafSize = 10.0;
    }

    if(leafSize == 0.0)
    {
        log_info( "Leafsize was 0, using 0.01" );
        leafSize = 0.01;
    }

    std::string target = params["target"].toString().toStdString();

    if ( env->getWorkspace()->getEntity(target) ) {
        // execute on entity
        return executeVoxelgrid(env, target, octreeLeafSize, leafSize);
    } else if ( env->getWorkspace()->getTree(target) ) {
        // execute on tree
        mapit::StatusCode status = MAPIT_STATUS_OK;
        env->getWorkspace()->depthFirstSearch(
                      target
                    , depthFirstSearchWorkspaceAll(mapit::msgs::Tree)
                    , depthFirstSearchWorkspaceAll(mapit::msgs::Tree)
                    , [&](std::shared_ptr<mapit::msgs::Entity> obj, const ObjectReference& ref, const mapit::Path &path)
                        {
                            status = executeVoxelgrid(env, path, octreeLeafSize, leafSize);
                            if ( ! mapitIsOk(status) ) {
                                return false;
                            }
                            return true;
                        }
                    , depthFirstSearchWorkspaceAll(mapit::msgs::Entity)
                    );
        return status;
    } else {
        log_error("operator voxelgrid: target is neither a tree nor entity");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }

    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME, "use pcl voxelgrid filter on a pointcloud", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, true, &operate_vxg)
