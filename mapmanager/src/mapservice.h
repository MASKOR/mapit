#ifndef __MAPSERVICE_H
#define __MAPSERVICE_H

#include "upns_globals.h"
#include "services.pb.h"
#include "abstractlayerdatastreamprovider.h"
#include "layerdata.h"
//#include "mapservice.h"
//#include "map.h"

namespace upns
{

using MapIdentifier = upnsuint64;
using LayerIdentifier = upnsuint64;

using MapVector = upnsVec< upnsSharedPointer<Map> >;
using LayerVector = upnsVec< upnsSharedPointer<Layer> >;

using MapResultsVector = upnsVec<upnsPair<MapIdentifier, int> >;

template<typename T>
bool upnsCheckResultVector( T result )
{
    return std::all_of(result.begin(), result.end(), [](typename T::value_type t){return t.second;});
}

/**
 * @brief The MapService class serves ability to read/write maps to/from a source.
 * Concrete implementations for sources like network, filesystem, ... exist.
 */

class MapService
{
public:
    virtual ~MapService() {};
    virtual upnsVec<MapIdentifier> listMaps() = 0;
    virtual MapVector getMaps(upnsVec<MapIdentifier> &mapIds) = 0;
    virtual MapResultsVector storeMaps( MapVector &maps ) = 0;
    virtual upnsSharedPointer<Map> createMap(upnsString name) = 0;
    virtual MapResultsVector removeMaps( upnsVec<MapIdentifier> &mapIds ) = 0;

    //virtual upnsSharedPointer<Map> receiveNewMap(upnsString name) = 0;

    virtual bool canRead() = 0;
    virtual bool canWrite() = 0;

    virtual upnsSharedPointer<AbstractLayerDataStreamProvider> getStreamProvider(MapIdentifier mapId, LayerIdentifier layerId) = 0;
};

}
#endif
