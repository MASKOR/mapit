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

#ifndef REG_GLOBAL_LUM_H
#define REG_GLOBAL_LUM_H

#include <memory>
#include <boost/shared_ptr.hpp>
#include <list>
#include <string>

#include <Eigen/Geometry>

#include <mapit/typedefs.h>
#include <mapit/time/time.h>
#include <mapit/registration_storage_helper/registration_storage_helper.h>

#include <pcl/registration/lum.h>

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

class RegGlobalLUM
{
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> Cloud;
    typedef Cloud::ConstPtr CloudConstPtr;
    typedef Cloud::Ptr CloudPtr;
    typedef std::pair<std::string, CloudPtr> CloudPair;
    typedef std::vector<CloudPair> CloudVector;

public:
    RegGlobalLUM(mapit::OperationEnvironment* env, mapit::StatusCode& status);

    mapit::StatusCode operate();
private:
    mapit::RegistrationStorageHelper* reg_helper_;

    bool elch_execute(  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> input
                     , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& target
                     , pcl::PointCloud<pcl::PointXYZ>& result_pc
                     , Eigen::Affine3f& result_transform
                     , double& fitness_score);

    bool loopDetection (int end, const CloudVector &clouds, double dist, int &first, int &last);

    pcl::registration::LUM<pcl::PointXYZ>::Ptr lum_;
};

}
#endif // REG_GLOBAL_LUM_H
