/*******************************************************************************
 *
 * Copyright      2017 Marcus Meeßen	<marcus.meessen@alumni.fh-aachen.de>
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
#include <mapit/layertypes/assettype.h>
#include <mapit/operators/versioning/checkoutraw.h>
#include <mapit/operators/operationenvironment.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <memory>
#include <mapit/errorcodes.h>
#include <mapit/operators/versioning/checkoutraw.h>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <tinyply.h>
#include <iostream>
#include <fstream>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/impl/supervoxel_clustering.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/mls.h>

#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>


// from pcl uniform sampling
struct Leaf
{
    Leaf () : idx (-1) { }
    int idx;
};

template <class PointT>
void voxelOffsetGridFilter(typename pcl::PointCloud<PointT>::Ptr &out,
                           typename pcl::PointCloud<PointT>::Ptr const &pointCloud,
                           std::double_t leafSize,
                           std::double_t offset)
{
    pcl::IndicesPtr indices(new std::vector<int>);
    for (std::uint64_t i = 0; i < pointCloud->size(); i++) {
        indices->push_back(i);
    }
    std::unordered_map<size_t, Leaf> leaves;


    out = typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    out->height = 1;
    out->is_dense = true;

    Eigen::Array4f inverse_leaf_size_ = Eigen::Array4f::Ones();
    inverse_leaf_size_[0] = 1.0 / leafSize;
    inverse_leaf_size_[1] = 1.0 / leafSize;
    inverse_leaf_size_[2] = 1.0 / leafSize;

    Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;
    Eigen::Vector4f min_p, max_p, offsetVector(offset, offset, offset, 0);
    // Get the minimum and maximum dimensions
    pcl::getMinMax3D<PointT>(*pointCloud, min_p, max_p);

    // Compute the minimum and maximum bounding box values
    min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
    max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));
    min_b_[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_[2]));
    max_b_[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_[2]));

    // Compute the number of divisions needed along all axis
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
    div_b_[3] = 0;

    // Set up the division multiplier
    divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);

    // First pass: build a set of leaves with the point index closest to the leaf center
    for (size_t cp = 0; cp < indices->size (); ++cp)
    {
        if (!pointCloud->is_dense)
            // Check if the point is invalid
            if (!pcl_isfinite (pointCloud->points[(*indices)[cp]].x) ||
                    !pcl_isfinite (pointCloud->points[(*indices)[cp]].y) ||
                    !pcl_isfinite (pointCloud->points[(*indices)[cp]].z))
                continue;

        Eigen::Vector4i ijk = Eigen::Vector4i::Zero ();
        ijk[0] = static_cast<int> (floor ((pointCloud->points[(*indices)[cp]].x + offset) * inverse_leaf_size_[0]));
        ijk[1] = static_cast<int> (floor ((pointCloud->points[(*indices)[cp]].y + offset) * inverse_leaf_size_[1]));
        ijk[2] = static_cast<int> (floor ((pointCloud->points[(*indices)[cp]].z + offset) * inverse_leaf_size_[2]));

        // Compute the leaf index
        int idx = (ijk - min_b_).dot (divb_mul_);
        Leaf& leaf = leaves[idx];
        // First time we initialize the index
        if (leaf.idx == -1)
        {
            leaf.idx = (*indices)[cp];
            continue;
        }

        // Check to see if this point is closer to the leaf center than the previous one we saved
        float diff_cur   = (pointCloud->points[(*indices)[cp]].getVector4fMap () - ijk.cast<float> ()).squaredNorm ();
        float diff_prev  = (pointCloud->points[leaf.idx].getVector4fMap ()       - ijk.cast<float> ()).squaredNorm ();

        // If current point is closer, copy its index instead
        if (diff_cur < diff_prev)
            leaf.idx = (*indices)[cp];
    }

    // Second pass: go over all leaves and copy data
    out->points.resize (leaves.size ());
    int cp = 0;

    for (typename std::unordered_map<size_t, Leaf>::const_iterator it = leaves.begin (); it != leaves.end (); ++it)
        out->points[cp++] = pointCloud->points[it->second.idx];
    out->width = static_cast<uint32_t> (out->points.size ());
}
// from pcl uniform sampling

void initialVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr &out,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr const pointCloud,
                      std::double_t const leafSize = 0.001)
{
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximateGrid;
    if (pointCloud->size() == 0) {
        throw std::runtime_error("point cloud is empty");
    }
    if (leafSize <= 0.0) {
        throw std::runtime_error("initial voxel grid size is smaller than or equal to zero");
    }
    log_info("initial voxel grid filter to high density of " << (leafSize * 1000.0) << "mm");
    out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    approximateGrid.setInputCloud(pointCloud);
    approximateGrid.setLeafSize(leafSize, leafSize, leafSize);
    approximateGrid.filter(*out);
    log_info("filter complete, " << (std::int64_t)pointCloud->size()
             - (std::int64_t)out->size() << " points removed");
    return;
}

void removeStatisticalOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr &out,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr const pointCloud,
                               std::double_t const deviation = 1.645) // keep >95% by default
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statisticalOutliers;
    if (pointCloud->size() == 0) {
        throw std::runtime_error("point cloud is empty");
    }
    if (deviation < 0.0) {
        throw std::runtime_error("deviation is smaller than zero");
    }
    log_info("remove statistical outliers with standard deviation of " << deviation << " sigma");
    out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    statisticalOutliers.setInputCloud(pointCloud);
    statisticalOutliers.setMeanK(20);
    statisticalOutliers.setStddevMulThresh(deviation);
    statisticalOutliers.filter(*out);
    log_info("filter complete, " << (std::int64_t)pointCloud->size()
             - (std::int64_t)out->size() << " points removed");
    return;
}

void removeRadiusOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr &out,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr const pointCloud,
                          std::double_t const radius,
                          std::uint32_t const neighbors)
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusOutliers;
    if (pointCloud->size() == 0) {
        throw std::runtime_error("point cloud is empty");
    }
    if (radius <= 0.0) {
        throw std::runtime_error("radius is smaller than or equal to zero");
    }
    log_info("remove radius outliers with less than " << neighbors << " neighbors in "
             << (radius * 1000.0) << "mm radius");
    out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    radiusOutliers.setInputCloud(pointCloud);
    radiusOutliers.setRadiusSearch(radius);
    radiusOutliers.setMinNeighborsInRadius(neighbors);
    radiusOutliers.filter(*out);
    log_info("filter complete, " << (std::int64_t)pointCloud->size()
             - (std::int64_t)out->size() << " points removed");
    return;
}

void movingLeastSquares(pcl::PointCloud<pcl::PointNormal>::Ptr &out,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr const pointCloud,
                        std::double_t const radius,
                        std::uint32_t const polynom = 5)
{
#if _OPENMP
    pcl::MovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointNormal> leastSquares;
#else
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> leastSquares;
#endif
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
    if (pointCloud->size() == 0) {
        throw std::runtime_error("point cloud is empty");
    }
    // smoothen surface, upsampling to close small holes
    out = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    kdTree->setSortedResults(true);
    kdTree->setInputCloud(pointCloud);
    leastSquares.setInputCloud(pointCloud);
    leastSquares.setComputeNormals(false);
    leastSquares.setPolynomialFit(true); // needed for upsampling!
    leastSquares.setPolynomialOrder(polynom);
    leastSquares.setSearchRadius(radius);
    leastSquares.setSearchMethod(kdTree);
    if (true) { // move decision to parameter
        log_info("upsampling");
        leastSquares.setUpsamplingMethod(
                    pcl::MovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
        leastSquares.setUpsamplingRadius(radius * .3);
        leastSquares.setUpsamplingStepSize(radius * .3);
    } else {
        leastSquares.setUpsamplingMethod(
                    pcl::MovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointNormal>::NONE);
    }
#if _OPENMP
    leastSquares.setNumberOfThreads(8);
#endif
    leastSquares.process(*out);
    log_info("moving least squares complete (" << (std::int64_t)pointCloud->size()
             - (std::int64_t)out->size() << " points)");
}

void finalVoxelGrid(pcl::PointCloud<pcl::PointNormal>::Ptr &out,
                    pcl::PointCloud<pcl::PointNormal>::Ptr const pointCloud,
                    std::double_t const radius)
{
    pcl::UniformSampling<pcl::PointNormal> uniformSampling;
    out = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    uniformSampling.setInputCloud(pointCloud);
    uniformSampling.setRadiusSearch(radius);
    uniformSampling.filter(*out);
    log_info("uniform sampling complete (" << (std::int64_t)pointCloud->size()
             - (std::int64_t)out->size() << " points)");
}

void filterCloud(std::double_t density,
                 std::double_t noise,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr in,
                 pcl::PointCloud<pcl::PointNormal>::Ptr out)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr pointCloudWithNormals;

    initialVoxelGrid(pointCloud, in);
    in->swap(*pointCloud);
    removeStatisticalOutliers(pointCloud, in);
    in->swap(*pointCloud);
    removeRadiusOutliers(pointCloud, in, (density + noise) * 1.5, 19);
    in->swap(*pointCloud);
    movingLeastSquares(pointCloudWithNormals, in, (density + noise) * 1.25);
    out->swap(*pointCloudWithNormals);
    finalVoxelGrid(pointCloudWithNormals, out, density);
    out->swap(*pointCloudWithNormals);

    // attempt to remove sticky neighbors
    pcl::search::KdTree<pcl::PointNormal> tree;
    tree.setInputCloud(out);
    std::uint32_t count = 0;
    std::for_each(out->begin(), out->end(), [&](pcl::PointNormal &point) {
        std::vector<std::int32_t> indices;
        std::vector<std::float_t> distances;
        tree.radiusSearch(point, density * 0.25, indices, distances);
        count += indices.size() - 1;
    });
    log_warn("false neighbors" << count);

    voxelOffsetGridFilter<pcl::PointNormal>(pointCloudWithNormals, out, density * 0.5, density * 0.25);
    out->swap(*pointCloudWithNormals);
    log_info("uniform sampling complete (" << (std::int64_t)pointCloudWithNormals->size()
             - (std::int64_t)out->size() << " points)");

    tree.setInputCloud(out);count = 0;
    std::for_each(out->begin(), out->end(), [&](pcl::PointNormal &point) {
        std::vector<std::int32_t> indices;
        std::vector<std::float_t> distances;
        tree.radiusSearch(point, density * 0.25, indices, distances);
        count += indices.size() - 1;
    });
    log_warn("false neighbors" << count);

    return;
}

void segmentCloud(std::double_t density,
                  std::double_t noise,
                  pcl::PointCloud<pcl::PointNormal>::Ptr const in,
                  std::vector<pcl::PointIndices> &out)
{
    pcl::EuclideanClusterExtraction<pcl::PointNormal> euclideanClustering;
    pcl::search::KdTree<pcl::PointNormal>::Ptr kdTree(new pcl::search::KdTree<pcl::PointNormal>());
    // generate segments by grouping spatial near points
    out.clear();
    kdTree->setSortedResults(true);
    kdTree->setInputCloud(in);
    euclideanClustering.setInputCloud(in);
    euclideanClustering.setClusterTolerance(density+noise);
    euclideanClustering.setMinClusterSize(5);
    euclideanClustering.setSearchMethod(kdTree);
    euclideanClustering.extract(out);
    log_info("euclidean clusters extracted (" << out.size() << " segments)");
    return;
}

void reconstructMesh(pcl::PointCloud<pcl::PointNormal>::Ptr /*const*/ &points,
                     std::vector<pcl::PointIndices> const &indices,
                     std::vector<std::vector<pcl::Vertices>> &out,
                     std::double_t const triangleSize)
{
    uint32_t completed = 0;
    pcl::search::KdTree<pcl::PointNormal>::Ptr kdTree(new pcl::search::KdTree<pcl::PointNormal>);
    kdTree->setSortedResults(true);
    kdTree->setInputCloud(points);
    pcl::PointCloud<pcl::PointNormal>::Ptr tempPointsVtk(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr tempPoints(new pcl::PointCloud<pcl::PointNormal>);

    out.clear();
    std::for_each(indices.begin(), indices.end(),
                  [&](pcl::PointIndices const &segment) -> void {
        if (completed > 50) return; // only use the biggest mesh for now

        log_info("begin segment" << std::to_string(completed));
        pcl::ConcaveHull<pcl::PointNormal> concaveHull;
        std::vector<pcl::Vertices> polygons;

        pcl::PointIndices hullPointIndices;

        // create concave hull of segment
        concaveHull.setInputCloud(points);
        concaveHull.setIndices(pcl::PointIndices::ConstPtr(new pcl::PointIndices(segment)));
        concaveHull.setAlpha(triangleSize);
        concaveHull.setDimension(3);
        concaveHull.setKeepInformation(true);
        concaveHull.setSearchMethod(kdTree);
        concaveHull.reconstruct(polygons);

        // revive old indices
        concaveHull.getHullPointIndices(hullPointIndices);

        for(auto polygon = polygons.begin(); polygon!=polygons.end(); polygon++)
        {
            polygon->vertices[0] = hullPointIndices.indices.at(polygon->vertices.at(0));
            polygon->vertices[1] = hullPointIndices.indices.at(polygon->vertices.at(1));
            polygon->vertices[2] = hullPointIndices.indices.at(polygon->vertices.at(2));
        }

        pcl::PolygonMesh tmpMeshIn, tmpMeshOut;
        tmpMeshIn.polygons = polygons;
        tmpMeshIn.cloud;
        pcl::toPCLPointCloud2(*points, tmpMeshIn.cloud);



        // save polygon if not empty
        if(polygons.size()>0 || tmpMeshOut.polygons.size() > 0) {


            pcl::MeshQuadricDecimationVTK vtk;
            pcl::MeshSmoothingLaplacianVTK vtkSmoothing;

            vtkSmoothing.setInputMesh(pcl::PolygonMesh::ConstPtr(new pcl::PolygonMesh(tmpMeshIn)));
            vtkSmoothing.setNumIter(5000);
            vtkSmoothing.setConvergence(0.0001);
            vtkSmoothing.setRelaxationFactor(0.0001);
            vtkSmoothing.setFeatureEdgeSmoothing(true);
            vtkSmoothing.setFeatureAngle(M_PI/5);
            vtkSmoothing.setBoundarySmoothing(true);

            vtkObject::GlobalWarningDisplayOff(); // pcl uses deprecated functions
            vtkSmoothing.process(tmpMeshOut);
            vtkObject::GlobalWarningDisplayOn();

            tmpMeshIn = tmpMeshOut;

            vtk.setInputMesh(pcl::PolygonMesh::ConstPtr(new pcl::PolygonMesh(tmpMeshIn)));
            vtk.setTargetReductionFactor(0.5);

            vtkObject::GlobalWarningDisplayOff(); // pcl uses deprecated functions
            vtk.process(tmpMeshOut);
            vtkObject::GlobalWarningDisplayOn();

            pcl::fromPCLPointCloud2(tmpMeshOut.cloud, *tempPointsVtk);



            // merge points
            std::uint32_t offset = tempPoints->size();
//            for(auto i = 0; i < tempPointsVtk->size(); i++) {
//                tempPoints->push_back(tempPointsVtk->at(i));
//            }
            (*tempPoints) += (*tempPointsVtk);
//            std::for_each(tempPointsVtk->begin(), tempPointsVtk->end(), [&](pcl::PointNormal const &point) {
//                tempPoints->push_back(point);
//            });
            std::for_each(tmpMeshOut.polygons.begin(), tmpMeshOut.polygons.end(), [&](pcl::Vertices &polygon){
                std::for_each(polygon.vertices.begin(), polygon.vertices.end(), [&](std::uint32_t &vertex){
                    vertex += offset;
                });
            });

            out.push_back(tmpMeshOut.polygons);
        } else {
            log_info("aborted" << std::to_string(completed));
            return;
        }

        completed++;
        log_info("completed" << std::to_string(completed));
        return;
    });

    points = tempPoints;

    log_info("meshes reconstructed (" << out.size() << " meshes)");
    return;
}

void simplifyMesh()
{
    // move vtk stuff to here
    return;
}

void pcl2tinyply(pcl::PointCloud<pcl::PointNormal>::Ptr const points,
                 std::vector<std::vector<pcl::Vertices>> const meshes,
                 tinyply::PlyFile &file)
{
    std::vector<std::float_t> pointsSerialized;
    //    std::vector<std::float_t> normalsSerialized;
    std::vector<std::int32_t> indicesSerialized;

    std::for_each(points->begin(), points->end(), [&](pcl::PointNormal const &point) -> void {
        pointsSerialized.push_back(point.x);
        pointsSerialized.push_back(point.y);
        pointsSerialized.push_back(point.z);
        //        normalsSerialized.push_back(point.normal_x);
        //        normalsSerialized.push_back(point.normal_y);
        //        normalsSerialized.push_back(point.normal_z);
    });

    std::for_each(meshes.begin(), meshes.end(), [&](std::vector<pcl::Vertices> const &mesh) -> void {
        std::for_each(mesh.begin(), mesh.end(), [&](pcl::Vertices const &vertices) -> void {
            std::for_each(vertices.vertices.begin(), vertices.vertices.end(), [&](std::int32_t const &vertex) -> void {
                indicesSerialized.push_back(vertex);
            });
        });
    });

    file.add_properties_to_element("vertex", {"x", "y", "z"}, pointsSerialized);
    //    file.add_properties_to_element("vertex", {"nx", "ny", "nz"}, normalsSerialized);
    file.add_properties_to_element("face", {"vertex_indices"}, indicesSerialized, 3, tinyply::PlyProperty::Type::INT32);
    file.comments.push_back("generated with resute");
    std::ofstream stream("/tmp/testfile.ply");
    file.write(stream, true);
}

mapit::StatusCode operate_surface_reconstruction(mapit::OperationEnvironment* environment)
{
    QByteArray parametersRaw(environment->getParameters().c_str(),
                             environment->getParameters().length());
    QJsonObject parameters(QJsonDocument::fromJson(parametersRaw).object());
    std::string target = parameters["target"].toString().toStdString();
    std::double_t density = parameters["density"].toDouble();
    std::double_t noise = parameters["noise"].toDouble();

    log_info("surface reconstruction begins");

    // get point cloud from environment
    std::shared_ptr<PointcloudEntitydata> entity_data =
            std::dynamic_pointer_cast<PointcloudEntitydata>(environment->getCheckout()->getEntitydataForReadWrite(target));
    if(entity_data == nullptr)
    {
        log_error("wrong type");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }

    std::shared_ptr<pcl::PCLPointCloud2> pointCloud2In = entity_data->getData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr pointCloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromPCLPointCloud2(*pointCloud2In, *pointCloudIn);

    filterCloud(density, noise, pointCloudIn, pointCloudWithNormals);
    log_info("filtering complete");
    std::vector<pcl::PointIndices> segments;
    segmentCloud(density * 1.5, noise, pointCloudWithNormals, segments);
    log_info("segmentation complete");
    std::vector<std::vector<pcl::Vertices>> meshes;
    reconstructMesh(pointCloudWithNormals, segments, meshes, density * 1.5);
    log_info("surface complete");
    tinyply::PlyFile file;
    // pcl2tinyply(pointCloudWithNormals, meshes, file);

    std::shared_ptr<pcl::PCLPointCloud2> pointCloud2Out(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*pointCloudWithNormals, *pointCloud2Out);

    // save filtered pointcloud
    std::shared_ptr<mapit::msgs::Entity> pointsEntity = environment->getCheckout()->getEntity(target + "_flt");
    if (pointsEntity == NULL) {
        pointsEntity = std::shared_ptr<mapit::msgs::Entity>(new mapit::msgs::Entity);
        pointsEntity->set_type(PointcloudEntitydata::TYPENAME());
        if (!mapitIsOk(environment->getCheckout()->storeEntity(target + "_flt", pointsEntity))) {
            log_error("Failed to create transform entity.");
            return MAPIT_STATUS_ERR_UNKNOWN;
        }
    }
    std::shared_ptr<mapit::AbstractEntitydata> abstractEntityDataPoints = environment->getCheckout()->getEntitydataForReadWrite( target + "_flt" );
    if (abstractEntityDataPoints == NULL) {
        return MAPIT_STATUS_ERR_UNKNOWN;
    }
    std::shared_ptr<PointcloudEntitydata> entityDataPoints = std::dynamic_pointer_cast<PointcloudEntitydata>( abstractEntityDataPoints );
    if (entityDataPoints == NULL) {
        log_error("Tf Transform has wrong type.");
        return MAPIT_STATUS_ERR_UNKNOWN;
    }
    entityDataPoints->setData(pointCloud2Out);

    // pcl2tinyply
    std::vector<std::float_t> pointsSerialized;
    std::vector<std::float_t> normalsSerialized;
    std::vector<std::int32_t> indicesSerialized;

    std::for_each(pointCloudWithNormals->begin(), pointCloudWithNormals->end(), [&](pcl::PointNormal const &point) -> void {
        pointsSerialized.push_back(point.x);
        pointsSerialized.push_back(point.y);
        pointsSerialized.push_back(point.z);
        normalsSerialized.push_back(point.normal_x);
        normalsSerialized.push_back(point.normal_y);
        normalsSerialized.push_back(point.normal_z);
    });

    std::for_each(meshes.begin(), meshes.end(), [&](std::vector<pcl::Vertices> const &mesh) -> void {
        std::for_each(mesh.begin(), mesh.end(), [&](pcl::Vertices const &vertices) -> void {
            std::for_each(vertices.vertices.begin(), vertices.vertices.end(), [&](std::int32_t const &vertex) -> void {
                indicesSerialized.push_back(vertex);
            });
        });
    });

    file.add_properties_to_element("vertex", {"x", "y", "z"}, pointsSerialized);
    file.add_properties_to_element("vertex", {"nx", "ny", "nz"}, normalsSerialized);
    file.add_properties_to_element("face", {"vertex_indices"}, indicesSerialized, 3, tinyply::PlyProperty::Type::UINT32);
    file.comments.push_back("generated with resute");

    // save filtered mesh
    std::shared_ptr<mapit::msgs::Entity> meshEntity = environment->getCheckout()->getEntity(target + "_msh");
    if (meshEntity == NULL) {
        meshEntity = std::shared_ptr<mapit::msgs::Entity>(new mapit::msgs::Entity);
        meshEntity->set_type(AssetEntitydata::TYPENAME());
        if (!mapitIsOk(environment->getCheckout()->storeEntity(target + "_msh", meshEntity))) {
            log_error("Failed to create transform entity.");
            return MAPIT_STATUS_ERR_UNKNOWN;
        }
    }
    std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydataMesh = environment->getCheckout()->getEntitydataForReadWrite( target + "_msh" );
    if (abstractEntitydataMesh == NULL) {
        return MAPIT_STATUS_ERR_UNKNOWN;
    }
    std::shared_ptr<AssetEntitydata> entityDataMesh = std::dynamic_pointer_cast<AssetEntitydata>( abstractEntitydataMesh );
    if (entityDataMesh == NULL) {
        log_error("Tf Transform has wrong type.");
        return MAPIT_STATUS_ERR_UNKNOWN;
    }
    AssetPtr tmp2(new AssetDataPair(file, nullptr));
    entityDataMesh->setData(tmp2);

    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME,
            "surface reconstruction",
            "Marcus Meeßen",
            OPERATOR_VERSION,
            PointcloudEntitydata_TYPENAME,
            &operate_surface_reconstruction)
