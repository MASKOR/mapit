#ifndef __POINTCLOUDLAYER_H__
#define __POINTCLOUDLAYER_H__

#include "abstractlayerdata.h"
#include "abstractlayerdatastreamprovider.h"
#include "layer.h"
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

using namespace upns;

class PointcloudLayerdata : public AbstractLayerData
{
    PointcloudLayerdata(AbstractLayerDataStreamProvider* streamProvider);

    Layer::LayerType layerType() const;

    LayerDataHandle getData(upnsReal x1, upnsReal y1, upnsReal z1,upnsReal x2, upnsReal y2, upnsReal z2, bool clipMode, int lod);
    int setData(upnsReal x1, upnsReal y1, upnsReal z1,upnsReal x2, upnsReal y2, upnsReal z2, LayerDataHandle &data, int lod);
private:
    AbstractLayerDataStreamProvider* m_streamProvider;
    //pcl::PointCloud<pcl::PointXYZ> m_pointcloud;
    pcl::PCLPointCloud2 m_pointcloud;
};

#endif
