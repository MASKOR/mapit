#include "pointcloudlayer.h"
#include <sstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

const char *PointcloudEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

PointcloudEntitydata::PointcloudEntitydata(upnsSharedPointer<AbstractEntitydataProvider> streamProvider)
    :m_streamProvider( streamProvider ),
     m_pointcloud( NULL )
{
}

const char *PointcloudEntitydata::type() const
{
    return PointcloudEntitydata::TYPENAME();
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
        ReadWriteHandle handle;
        upnsString filename = m_streamProvider->startReadFile(handle);
        {
            pcl::PCDReader reader;
            reader.read(filename, *m_pointcloud);
        }
        m_streamProvider->endReadFile(handle);
    }
    return m_pointcloud;
}

int PointcloudEntitydata::setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                 upnsReal x2, upnsReal y2, upnsReal z2,
                                 upnsPointcloud2Ptr &data,
                                 int lod)
{
    int result = -1;
    ReadWriteHandle handle;
    upnsString filename = m_streamProvider->startWriteFile(handle);
    {
        m_pointcloud = data;
        pcl::PCDWriter writer;
        result = writer.writeBinaryCompressed(filename, *m_pointcloud);
    }
    m_streamProvider->endWriteFile(handle);
    return result;
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
    return m_streamProvider->startRead(start, len);
}

void PointcloudEntitydata::endRead(upnsIStream *strm)
{
    m_streamProvider->endRead(strm);
}

upnsOStream *PointcloudEntitydata::startWriteBytes(upnsuint64 start, upnsuint64 len)
{
    return m_streamProvider->startWrite(start, len);
}

void PointcloudEntitydata::endWrite(upnsOStream *strm)
{
    m_streamProvider->endWrite(strm);
}

size_t PointcloudEntitydata::size() const
{
    m_streamProvider->getStreamSize();
}

// Win32 does not like anything but void pointers handled between libraries
// For Unix there would be a hack to use a "custom deleter" which is given to the library to clean up the created memory
// the common denominator is to build pointer with custom deleter in our main programm and just exchange void pointers and call delete when we are done
//upnsSharedPointer<AbstractEntitydata> createEntitydata(upnsSharedPointer<AbstractEntitydataProvider> streamProvider)
//void* createEntitydata(upnsSharedPointer<AbstractEntitydataProvider> streamProvider)
void deleteEntitydata(AbstractEntitydata *ld)
{
    PointcloudEntitydata *p = static_cast<PointcloudEntitydata*>(ld);
    delete p;
}
void createEntitydata(upnsSharedPointer<AbstractEntitydata> *out, upnsSharedPointer<AbstractEntitydataProvider> streamProvider)
{
    //return upnsSharedPointer<AbstractEntitydata>(new PointcloudEntitydata( streamProvider ), deleteWrappedLayerData);
    *out = upnsSharedPointer<AbstractEntitydata>(new PointcloudEntitydata( streamProvider ), deleteEntitydata);
}

