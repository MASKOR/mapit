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

}
