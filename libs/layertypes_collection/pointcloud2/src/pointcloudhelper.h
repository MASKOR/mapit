/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2015 Tobias Neumann	<t.neumann@fh-aachen.de>
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

//#ifndef POINTCLOUDHELPER_H
//#define POINTCLOUDHELPER_H

//#include <pcl/PCLPointCloud2.h>
//#define BOOST_NO_EXCEPTIONS 0
//#include <boost/archive/text_oarchive.hpp>
//#include <boost/archive/text_iarchive.hpp>
//#include <boost/serialization/vector.hpp>

//#include <pcl/io/pcd_io.h>

////template <typename PointT> int
////writeBinaryToStream (std::ostringstream &oss,
////                             const pcl::PointCloud<PointT> &cloud,
////                             const std::vector<int> &indices);

//int
//writeBinaryCompressed (std::ostream &oss, const pcl::PCLPointCloud2 &cloud,
//                                                const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (),
//                                                const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity ());


//int
//readPointcloudFromStream (std::istream &is, pcl::PCLPointCloud2 &cloud,
//                      Eigen::Vector4f origin = Eigen::Vector4f::Zero (),
//                      Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity (),
//                      int pcd_version = 0,
//                      const int offset = 0);

//namespace pcl {

//template<class Archive>
//void serialize(Archive & ar, ::pcl::PCLPointCloud2 &v, const unsigned int version)
//{
//    ar & v.header;
//    ar & v.height;
//    ar & v.width;

//    ar & v.fields;
//    ar & v.is_bigendian;
//    ar & v.point_step;
//    ar & v.row_step;
//    ar & v.data;
//    ar & v.is_dense;
//}

//template<class Archive>
//void serialize(Archive & ar, ::pcl::PCLHeader &header, const unsigned int version)
//{
//    ar & header.seq;
//    ar & header.stamp;
//    ar & header.frame_id;
//}

//template<class Archive>
//void serialize(Archive & ar, ::pcl::PCLPointField &f, const unsigned int version)
//{
//    ar & f.name;
//    ar & f.offset;
//    ar & f.datatype;
//    ar & f.count;
//}

//} // namespace pcl

//#endif


