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

#ifndef REGISTRATION_STORAGE_HELPER_H
#define REGISTRATION_STORAGE_HELPER_H

#include <memory>
#include <list>
#include <string>

#include <Eigen/Geometry>

#include <mapit/typedefs.h>
#include <mapit/time/time.h>

#include <functional>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class QJsonDocument;
class QJsonObject;

class PointcloudEntitydata;

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

class RegistrationStorageHelper
{
public:
    /**
     * @brief RegistrationStorageHelper get the configs, described below from the env
     * @param env
     *
     * structure:
     * {
     *  <string>"tf-prefix" : ..., // the prefix where to look for transforms (default "")
     *  <string>"input"[] : ..., // can either be, a list of entities or a tree containing enteties
     *  <string>"target" : ...,
     *
     *  optional <string>"frame_id" : ..., // all data given to the matching algorithm will be in this frame
     *
     *  <enum-as-string>"handle-result" : ["tf-add", "tf-combine", "data-change"],
     *  optional <string>"tf-frame_id" : ..., // in case of tf change or add
     *  optional <string>"tf-child_frame_id" : ..., // in case of tf change or add
     *  optional <bool>"tf-is_static" : ..., // in case of tf change or add (true only works with one input specified) (default false)
     *
     * }
     *
     * comments:
     * - tf-add:      will add the results of the matching algorithm without checking
     *                anything (old transforms (between "tf-frame_id"
     *                and "tf-child_frame_id") should probably been removed beforehand)
     * - tf-combine:  will combine the transforms in the system with the transforms
     *                from the matching algorithm but only at the times of the matching.
     *                That means afterwards the transforms between "tf-frame_id"
     *                and "tf-child_frame_id" are changed. In the time between the
     *                pointclouds given in the parameter "input" there will only
     *                be transforms at the time of each cloud (even so there was
     *                more transforms beforehand) these transforms will be the
     *                combination of the result of the matching algorithm and the
     *                previous transform at this time. Before and after the time of
     *                the pointclouds from "input" there will be no change.
     *                E.g. there are transforms at the times [0, 10, 20, 30, 40, 50],
     *                the stampes of the pointclouds in "input" are [19, 20, 31].
     *                This means the new transforms are [0, 10, 19-1ns, 19, 20, 31,
     *                31+1ns, 40, 50]. The transforms at 0, 10, 40 and 50 are the
     *                old transforms. The transforms at 19, 20, 31 are the new
     *                transforms * old transforms. And the transforms at 19-1ns
     *                and 31+1ns are the interpolated transforms based on the
     *                old transforms.
     *                This way, the old transformes are used from 0 - 19-1ns and
     *                31+1ns - 50. And the new transforms from 19 - 31.
     * - data-change: will change the data of the "input" clouds, tfs are not changed.
     */
    RegistrationStorageHelper(mapit::OperationEnvironment* env);

    /**
     * @brief get_pointcloud
     * @param path          in
     * @param stamp         out
     * @param header        out
     * @param entitydata    out
     * @throws mapit::StatusCode aka unsigned int
     * @return the pointcloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_pointcloud(  std::string path
                                                       , mapit::time::Stamp &stamp
                                                       , pcl::PCLHeader& header
                                                       , std::shared_ptr<PointcloudEntitydata>& entitydata
                                                      );

    /**
     * @brief operate_pairwise start the pairwise registration
     * @param algorithm std::function to the matching algorithm
     * @throws mapit::StatusCode aka unsigned int
     *
     * in case of a meta scan is requested, this can be done within
     * the algorithm function by modifing the target point cloud
     */
    void operate_pairwise(std::function<bool(  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input
                                             , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& target
                                             , pcl::PointCloud<pcl::PointXYZ>& result_pc
                                             , Eigen::Affine3f& result_transform
                                             , double& fitness_score)> algorithm);

    void operate_global(  std::function<void(pcl::PointCloud<pcl::PointXYZ>::Ptr)> callback_add_pointcloud
                        , std::function<void()> callback_search_and_process_loops
                        , std::function<void()> callback_execute_algorithm);

    // general setup
    mapit::operators::WorkspaceWritable* workspace_;
    std::shared_ptr<mapit::tf2::BufferCore> tf_buffer_;

    std::string cfg_tf_prefix_;

    std::list<std::string> cfg_input_;
    std::string cfg_target_;

    bool cfg_use_frame_id_;
    std::string cfg_frame_id_;

    enum class HandleResult {tf_add, tf_combine, data_change};
    HandleResult cfg_handle_result_;
    std::string cfg_tf_frame_id_;
    std::string cfg_tf_child_frame_id_;
    bool cfg_tf_is_static_;

private:
    /**
     * @brief mapit_add_tf
     * @param input_stamp
     * @param transform
     * @throws mapit::StatusCode aka unsigned int
     */
    void mapit_add_tf(  const time::Stamp &input_stamp
                      , const Eigen::Affine3f &transform
                     );

    /**
     * @brief mapit_remove_tfs
     * @param stamp_start
     * @param stamp_end
     * @throws mapit::StatusCode aka unsigned int
     */
    void mapit_remove_tfs(  const time::Stamp &stamp_start
                          , const time::Stamp &stamp_end
                         );
};

}
#endif // REGISTRATION_STORAGE_HELPER_H
