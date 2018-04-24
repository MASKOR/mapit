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

class RegistrationStorageHelper
{
public:
    RegistrationStorageHelper(mapit::OperationEnvironment* env);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> get_pointcloud(  std::string path
                                                                     , mapit::time::Stamp &stamp
                                                                     , pcl::PCLHeader& header
                                                                     , std::shared_ptr<PointcloudEntitydata> entitydata
                                                                    );

    void mapit_add_tf(  const time::Stamp &input_stamp
                      , const Eigen::Affine3f &transform
                     );
    void mapit_remove_tfs(  const time::Stamp &stamp_start
                          , const time::Stamp &stamp_end
                         );

public:
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
};

}
#endif // REGISTRATION_STORAGE_HELPER_H
