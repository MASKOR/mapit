#include "pointcloudlayer.h"
#include "pointcloudhelper.h"
#include <sstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

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
        m_pointcloud = upnsPointcloud2Ptr(new pcl::PCLPointCloud2);
        upnsIStream *in = m_streamProvider->startRead();
        {
            ::boost::archive::text_iarchive ia(*in);
            ia >> *m_pointcloud;
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
        ::boost::archive::text_oarchive oa(*out);
        oa << *data;
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
void deleteWrappedLayerData(AbstractEntityData* ld)
{
    delete ld;
}

//upnsSharedPointer<AbstractEntityData> createEntityData(upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
void* createEntityData(upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
{
    //return upnsSharedPointer<AbstractEntityData>(new PointcloudEntitydata( streamProvider ), deleteWrappedLayerData);
    return static_cast<void*>(new upnsSharedPointer<AbstractEntityData>(new PointcloudEntitydata( streamProvider ), deleteWrappedLayerData));
}
