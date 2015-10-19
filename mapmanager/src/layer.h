#ifndef __LAYER_H
#define __LAYER_H

#include "upns_globals.h"

namespace upns
{

using LayerIdentifier = upnsuint64;

class Layer
{
public:
    // keep in sync with .proto
    enum LayerType {
      POINTCLOUD2 = 0,
      OCTOMAP = 1,
      OPENVDB = 2
    };
    // keep in sync with .proto
    enum LayerUsageType {
      LASER = 0,
      RADAR = 1,
      NAVIGATION = 2,
      ANNOTATION = 3
    };
    LayerIdentifier layerId() const;
    void setLayerId(const LayerIdentifier &layerId);

    LayerType layerType() const;
    void setLayerType(const LayerType &layerType);

    LayerUsageType layerUsageType() const;
    void setLayerUsageType(const LayerUsageType &layerUsageType);

protected:
private:
    LayerIdentifier m_layerId;
    LayerType m_layerType;
    LayerUsageType m_layerUsageType;
};

}
#endif
