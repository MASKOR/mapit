#include "layer.h"
#include "upns.h"

namespace upns
{

LayerIdentifier Layer::layerId() const
{
    return m_layerId;
}

void Layer::setLayerId(const LayerIdentifier &layerId)
{
    m_layerId = layerId;
}

Layer::LayerType Layer::layerType() const
{
    return m_layerType;
}

void Layer::setLayerType(const LayerType &layerType)
{
    m_layerType = layerType;
}

Layer::LayerUsageType Layer::layerUsageType() const
{
    return m_layerUsageType;
}

void Layer::setLayerUsageType(const LayerUsageType &layerUsageType)
{
    m_layerUsageType = layerUsageType;
}

}
