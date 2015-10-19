#ifndef __MAP_H
#define __MAP_H

#include "upns_globals.h"
#include "layer.h"
#include "layerfilter.h"

namespace upns
{

using MapIdentifier = upnsuint64;


/**
 * \brief A map containing multiple layers, grouping them logically together.
 *
 * The layers of a map share the same coordinate system. If an operation on the map's layers results in a layer,
 * which remains in the same coordinate system, a layer will be added to the originating map (for most operators \sa Operator ).
 * If the layer is not in the same coordinate system, a new mayp will be created.
 *
 * This structure is the business object representation of maps. It acts as an interface for the developer to easily do operations and
 * not care too much about keeping things valid and in-sync.
 *
 * This class is not meant to do serialization, implementation of heavy pointcloud/computergraphics-algorithms.
 */

class Map
{
public:
    upnsString mapName() const;
    void setMapName(const upnsString &mapName);

    MapIdentifier mapId() const;
    void setMapId(const MapIdentifier &mapId);

    Layer& layer(LayerFilter filter) const;
    void addLayer(const Layer &layer);

    /**
     * Append all layers of rhs to the map.
     */
    Map& operator+=(const Map& rhs);
    friend Map operator+(Map lhs,
                      const Map& rhs);
    Layer& operator[](LayerFilter filter);;
protected:
private:
    upnsString m_mapName;
    MapIdentifier m_mapId;
    upnsVec<Layer> m_layer;
};

}
#endif
