#ifndef __POINTCLOUDHELPER_H__
#define __POINTCLOUDHELPER_H__

#include <pcl/PCLPointCloud2.h>

//#include <boost/archive/text_oarchive.hpp>
//#include <boost/archive/text_iarchive.hpp>
//#include <boost/serialization/vector.hpp>

namespace pcl {

template<class Archive>
void serialize(Archive & ar, ::pcl::PCLPointCloud2 &v, const unsigned int version)
{
    ar & v.header;
    ar & v.height;
    ar & v.width;

    ar & v.fields;
    ar & v.is_bigendian;
    ar & v.point_step;
    ar & v.row_step;
    ar & v.data;
    ar & v.is_dense;
}

template<class Archive>
void serialize(Archive & ar, ::pcl::PCLHeader &header, const unsigned int version)
{
    ar & header.seq;
    ar & header.stamp;
    ar & header.frame_id;
}

template<class Archive>
void serialize(Archive & ar, ::pcl::PCLPointField &f, const unsigned int version)
{
    ar & f.name;
    ar & f.offset;
    ar & f.datatype;
    ar & f.count;
}

} // namespace pcl

#endif


