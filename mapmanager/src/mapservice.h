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

using MapResultsVector = upnsVec<upnsPair<MapIdentifier, upnsuint32> >;

template<typename T>
bool upnsCheckResultVector( T result )
{
    return std::all_of(result.begin(), result.end(), [](typename T::value_type t){return t.second;});
}

/**
 * @brief The MapService class serves ability to read/write maps to/from a source without futher logic.
 * It can be seen as the abstraction of a file system. E.g. Versioning can be implemented on top of it.
 * E.g. 'remove' strictly deletes the physical representation of an entity without versioning it.
 * Concrete implementations for sources like network, filesystem, ... exist.
 * Layerdata Stream provider gives is part of this interface because the underlaying system must read/write layerdata as a stream.
 * To use layerdata as a user, a convenience class should be instanciated which gives easy access to the data in the layerspecific format
 * (e.g. you don't want to access parts of a stream but 'local regions' and 'areas').
 *
 * Maps/Layers use optimistic locking. If a write occurs and the system detects incompatible timestamps/versions, the store will fail
 *
 * Layerdata uses pessimistic. It allowes multiple reads and forbids simultaneous writes/readwrites.
 * The version system on top of the serialization must detect the locked layerdata and either:
 * 1. copy the data for write or
 * 2. calculate the physical representation of the layer again from history
 * There are locking methods for the system to detect collisions.
 * - Data locked for read,  read  access -> ok
 * - Data locked for read,  write access -> copy layerdata
 * - Data locked for write, read  access -> recalculate from history
 * - Data locked for write, write access -> recalculate from history
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

    virtual LockHandle lockLayerdataForRead(MapIdentifier mapId, LayerIdentifier layerId) = 0;
    virtual LockHandle lockLayerdataForWrite(MapIdentifier mapId, LayerIdentifier layerId) = 0;
    virtual void unlockLayerdataForWrite(LockHandle lockHandle) = 0;
    virtual void unlockLayerdataForRead(LockHandle lockHandle) = 0;

    virtual bool isLayerdataLockedForRead(MapIdentifier mapId, LayerIdentifier layerId) = 0;
    virtual bool isLayerdataLockedForWrite(MapIdentifier mapId, LayerIdentifier layerId) = 0;

    virtual bool canRead() = 0;
    virtual bool canWrite() = 0;

    virtual upnsSharedPointer<AbstractLayerDataStreamProvider> getStreamProvider(MapIdentifier mapId, LayerIdentifier layerId) = 0;
};

}
#endif
