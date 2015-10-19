#ifndef __ABSTRACTLAYERDATA_H
#define __ABSTRACTLAYERDATA_H

#include "upns_globals.h"
#include "layer.h"

namespace upns
{

/**
 * @brief The AbstractLayerData class is interface between a concrete layerdata implementation and layer. Basically an LayerData-Implementation will
 * translate/delegate requests of "getData" to LayerDataStreamProvider \sa AbstractLayerDataStreamProvider.
 */

class AbstractLayerData
{
public:
    virtual Layer::LayerType layerType() const = 0;

    virtual LayerDataHandle getData(upnsReal x1, upnsReal y1, upnsReal z1,upnsReal x2, upnsReal y2, upnsReal z2, bool clipMode, int lod) = 0;
    virtual int setData(upnsReal x1, upnsReal y1, upnsReal z1,upnsReal x2, upnsReal y2, upnsReal z2, LayerDataHandle &data, int lod) = 0;
};

}
#endif
