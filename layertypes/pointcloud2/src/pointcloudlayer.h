#ifndef __POINTCLOUDLAYER_H__
#define __POINTCLOUDLAYER_H__

#include "abstractlayerdata.h"
#include "abstractlayerdatastreamprovider.h"
#include <pcl/PCLPointCloud2.h>

using namespace upns;

using upnsPointcloud2Ptr = upnsSharedPointer<pcl::PCLPointCloud2>;

extern "C"
{
upnsSharedPointer<AbstractLayerData> createAbstractLayerData(upnsSharedPointer<AbstractLayerDataStreamProvider> streamProvider);
}

class PointcloudLayerdata : public LayerData<pcl::PCLPointCloud2>
{
public:
    PointcloudLayerdata(upnsSharedPointer<AbstractLayerDataStreamProvider> streamProvider);

    LayerType           layerType() const;
    bool                hasFixedGrid() const;
    bool                canSaveRegions() const;
    upnsPointcloud2Ptr  getData(upnsReal x1, upnsReal y1, upnsReal z1,upnsReal x2, upnsReal y2, upnsReal z2, bool clipMode, int lod = 0);
    int                 setData(upnsReal x1, upnsReal y1, upnsReal z1,upnsReal x2, upnsReal y2, upnsReal z2, upnsPointcloud2Ptr &data, int lod = 0);

    upnsPointcloud2Ptr  getData(int lod = 0);
    int                 setData(upnsPointcloud2Ptr &data, int lod = 0);

    void gridCellAt(upnsReal x, upnsReal y, upnsReal z, upnsReal &x1, upnsReal &y1, upnsReal &z1, upnsReal &x2, upnsReal &y2, upnsReal &z2) const;

private:
    upnsSharedPointer<AbstractLayerDataStreamProvider> m_streamProvider;
    //pcl::PointCloud<pcl::PointXYZ> m_pointcloud;
    upnsPointcloud2Ptr m_pointcloud;

};

#endif
