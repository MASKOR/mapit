#include "pointcloudlayer.h"
#include "pointcloudhelper.h"
#include <sstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

template <typename T>
class not_deleter {
public:
  void operator()(T* ptr){}
};

PointcloudEntitydata::PointcloudEntitydata(upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
    :m_streamProvider( streamProvider ),
     m_pointcloud( NULL )
{
}

LayerType PointcloudEntitydata::layerType() const
{
    return LayerType::POINTCLOUD2;
}

bool PointcloudEntitydata::hasFixedGrid() const
{
    return false;
}

bool PointcloudEntitydata::canSaveRegions() const
{
    return false;
}

upnsPointcloud2Ptr PointcloudEntitydata::getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                                upnsReal x2, upnsReal y2, upnsReal z2,
                                                bool clipMode,
                                                int lod)
{
    if(m_pointcloud == NULL)
    {
        m_pointcloud = upnsPointcloud2Ptr(new ::pcl::PCLPointCloud2);
        upnsIStream *in = m_streamProvider->startRead();
        {
            //TODO
//            ::boost::archive::text_iarchive ia(*in);
//            ia >> *m_pointcloud;
            readPointcloudFromStream( *in, *m_pointcloud );
//            std::copy(myVector.begin(), myVector.end(), std::ostreambuf_iterator<char>(FILE));

//            std::istreambuf_iterator iter(in);
//            std::copy(iter.begin(), iter.end(), std::back_inserter(m_pointcloud->fields));
//            std::copy(iter.begin(), iter.end(), std::back_inserter(m_pointcloud->data));
//            in->read(reinterpret_cast<char*>(m_pointcloud.get()), sizeof(::pcl::PCLPointCloud2));
        }
        m_streamProvider->endRead(in);
    }
    return m_pointcloud;
}

int PointcloudEntitydata::setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                 upnsReal x2, upnsReal y2, upnsReal z2,
                                 upnsPointcloud2Ptr &data,
                                 int lod)
{
    upnsOStream *out = m_streamProvider->startWrite();
    {
        //TODO
//        out << *data;
//        out->write(reinterpret_cast<char*>(data.get()), sizeof(::pcl::PCLPointCloud2));
//        ::boost::archive::text_oarchive oa(*out);
//        oa << *data;
//        pcl::io::OctreePointCloudCompression<pcl::PCLPointCloud2> encoder;
//        boost::shared_ptr<const pcl::PCLPointCloud2> ptr(data.get(), not_deleter<pcl::PCLPointCloud2>());
//        encoder.encodePointCloud(ptr, *out);
        writeBinaryCompressed( *out, *data );
    }
    m_streamProvider->endWrite(out);
	return 0; //TODO: MSVC: What to return here?
}

upnsPointcloud2Ptr PointcloudEntitydata::getData(int lod)
{
    return getData(-std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                   false, lod);
}

int PointcloudEntitydata::setData(upnsPointcloud2Ptr &data, int lod)
{
    return setData(-std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                   data, lod);
}

void PointcloudEntitydata::gridCellAt(upnsReal   x, upnsReal   y, upnsReal   z,
                                     upnsReal &x1, upnsReal &y1, upnsReal &z1,
                                     upnsReal &x2, upnsReal &y2, upnsReal &z2) const
{
    x1 = -std::numeric_limits<upnsReal>::infinity();
    y1 = -std::numeric_limits<upnsReal>::infinity();
    z1 = -std::numeric_limits<upnsReal>::infinity();
    x2 = +std::numeric_limits<upnsReal>::infinity();
    y2 = +std::numeric_limits<upnsReal>::infinity();
    z2 = +std::numeric_limits<upnsReal>::infinity();
}

int PointcloudEntitydata::getEntityBoundingBox(upnsReal &x1, upnsReal &y1, upnsReal &z1,
                                              upnsReal &x2, upnsReal &y2, upnsReal &z2)
{
    //TODO
    return 0;
}

upnsIStream *PointcloudEntitydata::startReadBytes(upnsuint64 start, upnsuint64 len)
{
    m_streamProvider->startRead(start, len);
}

void PointcloudEntitydata::endRead(upnsIStream *strm)
{
    m_streamProvider->endRead(strm);
}
//void deleteEntitydata(void* ld)
void deleteEntitydata(AbstractEntityData *ld)
{
    PointcloudEntitydata *p = static_cast<PointcloudEntitydata*>(ld);
    delete p;
}
// Win32 does not like anything but void pointers handled between libraries
// For Unix there would be a hack to use a "custom deleter" which is given be the library to clean up the created memory
// the common denominator is to build pointer with custom deleter in our main programm and just exchange void pointers and call delete when we are done
//upnsSharedPointer<AbstractEntityData> createEntitydata(upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
//void* createEntitydata(upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
void createEntitydata(upnsSharedPointer<AbstractEntityData> *out, upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
{
    //return upnsSharedPointer<AbstractEntityData>(new PointcloudEntitydata( streamProvider ), deleteWrappedLayerData);
    *out = upnsSharedPointer<AbstractEntityData>(new PointcloudEntitydata( streamProvider ), deleteEntitydata);
}
