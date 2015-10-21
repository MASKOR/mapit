#ifndef __MAPSERVICE_H
#define __MAPSERVICE_H

#include "upns_globals.h"
#include "map.h"

namespace upns
{

using MapIdentifier = upnsuint64;
using LayerIdentifier = upnsuint64;

using MapVector = upnsVec< upnsSharedPointer<Map> >;
using LayerVector = upnsVec< upnsSharedPointer<Layer> >;

/**
 * @brief The MapService class serves ability to read/write maps to/from a source.
 * Concrete implementations for sources like network, filesystem, ... exist.
 */

class MapService
{
    virtual upnsVec<MapIdentifier> listMaps() = 0;
    virtual MapVector getMaps(upnsVec<MapIdentifier> &mapIds) = 0;
    virtual upnsVec<upnsPair<MapIdentifier, int> > storeMaps( MapVector &maps ) = 0;
    virtual upnsSharedPointer<Map> createMap(upnsString &name) = 0;
};

}
#endif
