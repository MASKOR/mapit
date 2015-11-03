#ifndef __POINTCLOUDHELPER_H__
#define __POINTCLOUDHELPER_H__

#include <pcl/PCLPointCloud2.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

//namespace upns
//{

//int readPointcloud2(std::istream &fs, pcl::PCLPointCloud2 &cloud);

//int readPointcloud2New(std::istream& s, const  ::pcl::PCLPointCloud2 &v);

//int writePointcloud2New(std::ostream& s, const  ::pcl::PCLPointCloud2 &v);

namespace pcl {

template<class Archive>
void serialize(Archive & ar, ::pcl::PCLPointCloud2 &v, const unsigned int version)
{
    ar & v.header;
    ar & v.height;
    ar & v.width;

    ar & v.fields;
//    size_t fieldssize;
//    fieldssize << s;
//    for (size_t i = 0; i < fieldssize; ++i)
//    {
//        ::pcl::PCLPointField f;
//        s >> f;
//        v.fields.push_back(f);
//    }
    ar & v.is_bigendian;
    ar & v.point_step;
    ar & v.row_step;
    ar & v.data;
//    size_t datasize;
//    s >> datasize;
//    for (size_t i = 0; i < datasize; ++i)
//    {
//        ::pcl::uint8_t d;
//        s >> d;
//        v.data.push_back(d);
//    }
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

} // namespace boost

//}
#endif


