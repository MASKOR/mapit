#include "pointcloudlayer.h"
#include "pointcloudhelper.h"

PointcloudLayerdata::PointcloudLayerdata(upns::AbstractLayerDataStreamProvider *streamProvider)
    :m_streamProvider( streamProvider ),
     m_pointcloud( NULL )
{
}

UpnsLayerType PointcloudLayerdata::layerType() const
{
    return UpnsLayerType::POINTCLOUD2;
}

bool PointcloudLayerdata::hasFixedGrid() const
{
    return false;
}

bool PointcloudLayerdata::canSaveRegions() const
{
    return false;
}

upnsPointcloud2Ptr PointcloudLayerdata::getData(upnsReal x1, upnsReal y1, upnsReal z1, upnsReal x2, upnsReal y2, upnsReal z2, bool clipMode, int lod)
{
    if(m_pointcloud == NULL)
    {
        m_pointcloud = upnsPointcloud2Ptr(new pcl::PCLPointCloud2);
        upnsIStream &in = m_streamProvider->startRead();
        readPointcloud2(in, *m_pointcloud);
        m_streamProvider->endRead(in);
    }
    return m_pointcloud;
}

int PointcloudLayerdata::setData(upnsReal x1, upnsReal y1, upnsReal z1, upnsReal x2, upnsReal y2, upnsReal z2, upnsPointcloud2Ptr &data, int lod)
{
    upnsOStream &out = m_streamProvider->startWrite();
    out << data.get();
    m_streamProvider->endWrite(out);
}
