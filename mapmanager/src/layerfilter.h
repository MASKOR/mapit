#ifndef __LAYERFILTER_H
#define __LAYERFILTER_H

#include "upns_globals.h"
#include "layer.h"

namespace upns
{
class Map;

class LayerFilter
{
public:
    // internal filter to address layer directly
    LayerFilter(LayerIdentifier id);
    LayerFilter(Layer::LayerType layerType);
    LayerFilter(Layer::LayerUsageType layerUsageType);

    //TODO: not needed?
    //operator LayerIdentifier() const { return LayerFilter(); }

    LayerFilter& operator&(const LayerIdentifier id) const;
    LayerFilter& operator&(const Layer::LayerType layerType) const;
    LayerFilter& operator&(const Layer::LayerUsageType layerUsageType) const;
    LayerFilter& operator&(const LayerFilter& filter) const;
    LayerFilter& operator|(const LayerIdentifier id) const;
    LayerFilter& operator|(const Layer::LayerType layerType) const;
    LayerFilter& operator|(const Layer::LayerUsageType layerUsageType) const;
    LayerFilter& operator|(const LayerFilter& filter) const;

    Layer& retrieve(const Map& map);
    Layer* retrieveIfUnique(const Map& map);
    upnsVec<Layer*> retrieveAll(const Map& map);
    bool isUniqueLayer(const Map& map);

    Layer& retrieve(const upnsVec<Map*> maps);
    Layer* retrieveIfUnique(const upnsVec<Map*> maps);
    upnsVec<Layer*> retrieveAll(const upnsVec<Map*> &maps);
    bool isUniqueLayer(const upnsVec<Map*> &maps);
protected:
private:
};

}
#endif
