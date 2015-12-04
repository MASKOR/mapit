#ifndef __POINTCLOUDLAYER_H__
#define __POINTCLOUDLAYER_H__

#include "entitydata.h"
#include "abstractentitydatastreamprovider.h"
#include <pcl/PCLPointCloud2.h>

using namespace upns;

using upnsPointcloud2Ptr = upnsSharedPointer<pcl::PCLPointCloud2>;

extern "C"
{
upnsSharedPointer<AbstractEntityData> createEntityData(upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider);
}

class PointcloudEntitydata : public EntityData<pcl::PCLPointCloud2>
{
public:
    PointcloudEntitydata(upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider);

    LayerType           layerType() const;
    bool                hasFixedGrid() const;
    bool                canSaveRegions() const;
    upnsPointcloud2Ptr  getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                bool clipMode,
                                int lod = 0);
    int                 setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                upnsPointcloud2Ptr &data,
                                int lod = 0);

    upnsPointcloud2Ptr  getData(int lod = 0);
    int                 setData(upnsPointcloud2Ptr &data, int lod = 0);

    void gridCellAt(upnsReal   x, upnsReal   y, upnsReal   z,
                    upnsReal &x1, upnsReal &y1, upnsReal &z1,
                    upnsReal &x2, upnsReal &y2, upnsReal &z2) const;

    int getEntityBoundingBox(upnsReal &x1, upnsReal &y1, upnsReal &z1,
                             upnsReal &x2, upnsReal &y2, upnsReal &z2);

private:
    upnsSharedPointer<AbstractEntityDataStreamProvider> m_streamProvider;
    //pcl::PointCloud<pcl::PointXYZ> m_pointcloud;
    upnsPointcloud2Ptr m_pointcloud;

};

#endif
