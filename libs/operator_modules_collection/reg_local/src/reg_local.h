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

#ifndef ICP_H
#define ICP_H

#include <memory>
#include <boost/shared_ptr.hpp>
#include <list>
#include <string>

#include <Eigen/Geometry>

#include <mapit/typedefs.h>
#include <mapit/time/time.h>

class QJsonDocument;
class QJsonObject;

class PointcloudEntitydata;
namespace pcl {
template <typename PointT>
class PointCloud;
struct PointXYZ;
struct PCLHeader;
}

namespace mapit {
class OperationEnvironment;
namespace operators
{
class WorkspaceWritable;
}
}

namespace mapit {
namespace tf2 {
class BufferCore;
}

class RegLocal
{
public:
    RegLocal(mapit::OperationEnvironment* env, mapit::StatusCode& status);

    mapit::StatusCode operate();
private:
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> get_pointcloud(  std::string path
                                                                     , mapit::StatusCode& status
                                                                     , mapit::time::Stamp &stamp
                                                                     , pcl::PCLHeader& header
                                                                     , std::shared_ptr<PointcloudEntitydata> entitydata
                                                                    );

    mapit::StatusCode mapit_add_tf(  const time::Stamp &input_stamp
                                  , const Eigen::Affine3f &transform
                                 );
    mapit::StatusCode mapit_remove_tfs(  const time::Stamp &stamp_start
                                      , const time::Stamp &stamp_end
                                     );

    // general setup
    mapit::operators::WorkspaceWritable* workspace_;
    std::shared_ptr<mapit::tf2::BufferCore> tf_buffer_;

    std::string cfg_tf_prefix_;

    std::list<std::string> cfg_input_;
    std::string cfg_target_;

    bool cfg_use_frame_id_;
    std::string cfg_frame_id_;
    bool cfg_use_metascan_;

    enum class HandleResult {tf_add, tf_combine, data_change};
    HandleResult cfg_handle_result_;
    std::string cfg_tf_frame_id_;
    std::string cfg_tf_child_frame_id_;
    bool cfg_tf_is_static_;

    //--- algorithm configs ---
    enum class MatchingAlgorithm {ICP};
    MatchingAlgorithm cfg_matching_algorithm_;
    // ICP
    mapit::StatusCode get_cfg_icp(const QJsonObject& params);
    void icp_execute(  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input
                     , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> target
                     , pcl::PointCloud<pcl::PointXYZ>& result_pc
                     , Eigen::Affine3f& result_transform
                     , bool& has_converged
                     , double& fitness_score);

    bool cfg_icp_set_maximum_iterations_;
    int cfg_icp_maximum_iterations_;
    bool cfg_icp_set_max_correspondence_distance_;
    double cfg_icp_max_correspondence_distance_;
    bool cfg_icp_set_transformation_epsilon_;
    double cfg_icp_transformation_epsilon_;
    bool cfg_icp_set_euclidean_fitness_epsilon_;
    double cfg_icp_euclidean_fitness_epsilon_;

};

}
#endif // ICP_H
