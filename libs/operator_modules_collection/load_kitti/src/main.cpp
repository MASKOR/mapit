/*******************************************************************************
 *
 * Copyright 2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include <mapit/time/time.h>

#include <mapit/layertypes/tflayer.h>

#include <mapit/layertypes/pointcloudlayer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
namespace fs = boost::filesystem;

#include <boost/algorithm/string.hpp>
#include <vector>
#include <algorithm>
#include <iterator>
#include <fstream>

void
get_or_create_entity(  mapit::operators::WorkspaceWritable* workspace
                                      , const std::string& entity_name
                                      , const std::string& entity_type
                                      , std::shared_ptr<mapit::msgs::Entity>& entity)
{
    entity = workspace->getEntity(entity_name);
    if (entity == nullptr) {
        entity = std::make_shared<mapit::msgs::Entity>();
        entity->set_type( entity_type );
        workspace->storeEntity(entity_name, entity);
    }
    if ( 0 != entity->type().compare( entity_type ) ) {
        throw MAPIT_STATUS_ERROR;
    }
}

double
latitude_to_scale(const double& latitude)
{
    return cos(latitude * M_PI / 180.0);
}

void
latlon_to_mercator(  const double& latitude
                   , const double& longitude
                   , const double& scale
                   , double& x
                   , double& y)
{
    double er = 6378137;
    x = scale * longitude * M_PI * er / 180;
    y = scale * er * log( tan((90+latitude) * M_PI / 360) );
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>
read_kitti_pointcloud(fs::path file_name)
{
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> pc = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    float *data = (float*)malloc(num*sizeof(float));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pi = data+3;

    // load point cloud
    FILE *stream;
    stream = fopen(file_name.string().c_str(), "rb");
    num = fread(data, sizeof(float), num, stream) / 4;
    for (int32_t i = 0; i < num; ++i) {
        pcl::PointXYZI p;
        p.x = *px;
        p.y = *py;
        p.z = *pz;
        p.intensity = *pi * 100;
        pc->push_back( p );

        px+=4;
        py+=4;
        pz+=4;
        pi+=4;
    }
    fclose(stream);

    return pc;
}

mapit::tf::TransformPtr
read_kitti_transform(fs::path file_name, double& scale, const bool& is_first, const mapit::tf::TransformPtr& origin)
{
    mapit::tf::TransformPtr ret = std::make_shared<mapit::tf::Transform>();
    std::string line;
    std::ifstream myfile( file_name.string().c_str() );
    if ( myfile.is_open() ) {
        if ( std::getline(myfile, line) ) {
            std::vector<std::string> strs;
            boost::split(strs, line, boost::is_any_of("\t "));

            // get needed data from file
            double latitude = std::stod( strs.at(0) );
            double longitude = std::stod( strs.at(1) );
            double z = std::stod( strs.at(2) );
            double roll = std::stod( strs.at(3) );
            double pitch = std::stod( strs.at(4) );
            double yaw = std::stod( strs.at(5) );

            // convert data to transform matrix
            if (is_first) {
                scale = latitude_to_scale( latitude );
                log_info("load_kitti: Mercator scale: " << scale);
            }
            double x, y;
            latlon_to_mercator(latitude, longitude, scale, x, y);

            ret->rotation = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
                          * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
                          * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
            ret->rotation = ret->rotation * origin->rotation.inverse();
            ret->translation = Eigen::Translation3f(x, y, z);
            ret->translation = Eigen::Translation3f(ret->translation.vector() - origin->translation.vector());
        }
        myfile.close();
    }

    return ret;
}

void
store_pointcloud(  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> pc
                 , const std::string& prefix_pointclouds
                 , const std::string& pc_frame_id
                 , const mapit::time::Stamp& stamp
                 , mapit::operators::WorkspaceWritable* workspace)
{
    std::string entity_name = prefix_pointclouds + "/velodyne_points_" + mapit::time::to_string(stamp);

    std::shared_ptr<mapit::msgs::Entity> entity;
    try {
        get_or_create_entity(workspace, entity_name, PointcloudEntitydata::TYPENAME(), entity);
    } catch (mapit::StatusCode err) {
        log_error("load_kitti: Can't create entity \"" << entity_name << "\" of type \"" << PointcloudEntitydata::TYPENAME() << "\", error: " << err);
        throw err;
    }
    entity->set_frame_id( pc_frame_id );
    entity->set_allocated_stamp( mapit::time::to_msg_allocated(stamp) );

//    log_info("load_kitti:   entity: " + entity_name);
    workspace->storeEntity(entity_name, entity);

    // write entity data
    std::shared_ptr<pcl::PCLPointCloud2> entity_data = std::make_shared<pcl::PCLPointCloud2>();
    pcl::toPCLPointCloud2(*pc, *entity_data);
    std::static_pointer_cast<PointcloudEntitydata>(workspace->getEntitydataForReadWrite(entity_name))->setData(entity_data);
}

std::string escape_slashes(const std::string& in)
{
   std::string out = in;
   std::replace( out.begin(), out.end(), '/', '_');
   return out;
}

mapit::time::Stamp
stamp_from_stream(std::ifstream& stream)
{
    std::string stamp_line;
    std::getline(stream, stamp_line);
    return mapit::time::from_string( stamp_line );
}

mapit::StatusCode operate_load_kitti(mapit::OperationEnvironment* env)
{
    /** structure:
     * {
     *   "datasets" : ["name_of_dataset_1", ...],
     *   "prefix-transforms" : <string>prefix where to store the transforms,
     *   "transform-frame_id" : <string>frame_id for transforms,
     *   "transform-intermediate_frame_id" : <string>frame_id for offset,
     *   "transform-child_frame_id" : <string>child_frame_id for transforms,
     *   "prefix-pointclouds" : <string>prefix where to store the pointclouds
     *   "pointcloud-frame_id" : <string>frame_id for pointclouds
     *    TODO import camera data/pictures
     * }
     */
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    mapit::operators::WorkspaceWritable* workspace = env->getWorkspace();

    std::string prefix_pointclouds = params["prefix-pointclouds"].toString().toStdString();
    std::string prefix_transforms = params["prefix-transforms"].toString().toStdString();
    log_info("load_kitti: prefix-pointclouds:              " << prefix_pointclouds);
    log_info("load_kitti: prefix-transforms:               " << prefix_transforms);

    std::string tf_frame_id = params["transform-frame_id"].toString().toStdString();
    std::string tf_child_frame_id = params["transform-child_frame_id"].toString().toStdString();
    std::string tf_intermediate_frame_id = params["transform-intermediate_frame_id"].toString().toStdString();
    log_info("load_kitti: transform-frame_id:              " << tf_frame_id);
    log_info("load_kitti: transform-child_frame_id:        " << tf_child_frame_id);
    log_info("load_kitti: transform-intermediate_frame_id: " << tf_intermediate_frame_id);

    std::string pc_frame_id = params["pointcloud-frame_id"].toString().toStdString();
    log_info("load_kitti: pointcloud-frame_id:             " << pc_frame_id);

    QJsonArray json_dataset_names( params["datasets"].toArray() );
    for (auto json_dataset_name : json_dataset_names) {
        std::string dataset_name = json_dataset_name.toString().toStdString();

        log_info("load_kitti: Open dataset " + dataset_name);

        fs::path _TRANSFORMS_  = "oxts";
        fs::path _POINTCLOUDS_ = "velodyne_points";
        fs::path dataset_path = dataset_name;

        // --- transforms
        // open time file
        fs::path tf_time = dataset_path / _TRANSFORMS_ / "timestamps.txt";
        std::ifstream tf_time_file(tf_time.string().c_str());

        // sort all transforms
        std::vector<fs::path> v_sorted_tfs;
        std::copy(fs::directory_iterator( dataset_path / _TRANSFORMS_ / "data" ), fs::directory_iterator(), std::back_inserter(v_sorted_tfs));
        std::sort(v_sorted_tfs.begin(), v_sorted_tfs.end());

        // for all transforms
        bool is_first = true;
        double scale = 0;
        mapit::tf::TransformPtr origin = std::make_shared<mapit::tf::Transform>();
        origin->Identity();
        std::shared_ptr<mapit::tf::store::TransformStampedListGatherer> tfs_map
                = std::make_shared<mapit::tf::store::TransformStampedListGatherer>();
        for( fs::path file : v_sorted_tfs ) {
//            log_info("load file: " << file.string());
            mapit::tf::TransformPtr tf = read_kitti_transform( file, scale, is_first, origin );

            // read time stamp
            mapit::time::Stamp stamp = stamp_from_stream(tf_time_file);

            if (is_first) {
                *origin = *tf;
                is_first = false;

                std::unique_ptr<mapit::tf::TransformStamped> origin_stamped = std::make_unique<mapit::tf::TransformStamped>();
                origin_stamped->frame_id = tf_frame_id;
                origin_stamped->child_frame_id = tf_intermediate_frame_id;
                origin_stamped->stamp = stamp;

                origin_stamped->transform = *origin;

                log_info("Origin: " << origin_stamped->frame_id << " -> " << origin_stamped->child_frame_id
                         << " with "
                         << "t = (" << origin_stamped->transform.translation.x() << ", "
                                    << origin_stamped->transform.translation.y() << ", "
                                    << origin_stamped->transform.translation.z() << ") "
                         << "q = (" << origin_stamped->transform.rotation.w() << ", "
                                    << origin_stamped->transform.rotation.x() << ", "
                                    << origin_stamped->transform.rotation.y() << ", "
                                    << origin_stamped->transform.rotation.z() << ")");

                tfs_map->add_transform( std::move(origin_stamped), true);

                tf->Identity();
            }

            // get to tfs map
            std::unique_ptr<mapit::tf::TransformStamped> tf_loaded = std::make_unique<mapit::tf::TransformStamped>();
            tf_loaded->frame_id = tf_intermediate_frame_id;
            tf_loaded->child_frame_id = tf_child_frame_id;
            tf_loaded->stamp = stamp;
            tf_loaded->transform = *tf;

//            log_info("TF: " << tf_loaded->frame_id << " -> " << tf_loaded->child_frame_id
//                     << " with "
//                     << "t = (" << tf_loaded->transform.translation.x() << ", "
//                                << tf_loaded->transform.translation.y() << ", "
//                                << tf_loaded->transform.translation.z() << ") "
//                     << "q = (" << tf_loaded->transform.rotation.w() << ", "
//                                << tf_loaded->transform.rotation.x() << ", "
//                                << tf_loaded->transform.rotation.y() << ", "
//                                << tf_loaded->transform.rotation.z() << ")");

            tfs_map->add_transform( std::move( tf_loaded ), false );
        }

        // save tf map to mapit
        mapit::StatusCode status = tfs_map->store_entities(workspace, prefix_transforms);
        if ( ! mapitIsOk(status) ) {
            log_error("load_kitti: error " << status << " while storing transforms");
            return status;
        }

        // --- POINTCLOUDS
        // open time file
        fs::path pointcloud_time = dataset_path / _POINTCLOUDS_ / "timestamps.txt";
        std::ifstream pointcloud_time_file(pointcloud_time.string().c_str());

        // sort all pointclouds
        std::vector<fs::path> v_sorted_pcs;
        std::copy(fs::directory_iterator( dataset_path / _POINTCLOUDS_ / "data" ), fs::directory_iterator(), std::back_inserter(v_sorted_pcs));
        std::sort(v_sorted_pcs.begin(), v_sorted_pcs.end());

        // for all pointclouds
        for( fs::path file : v_sorted_pcs) {
            // read datafile
//            log_info("load file: " << file.string());
            std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> pc = read_kitti_pointcloud(file);

//            // read time stamp
//            mapit::time::Stamp stamp;
//            std::string stamp_line;
//            std::getline(pointcloud_time_file, stamp_line);
//            stamp = mapit::time::from_string( stamp_line );
            // read time stamp
            mapit::time::Stamp stamp = stamp_from_stream(pointcloud_time_file);

            // create entit
            try {
                store_pointcloud(pc, prefix_pointclouds, pc_frame_id, stamp, workspace);
            } catch(mapit::StatusCode err) {
                return err;
            }
        }

    }

    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME, "load bagfiles in mapit", "fhac", OPERATOR_VERSION, "any", false, &operate_load_kitti)
