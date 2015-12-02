#ifndef __MAPSERVICE_H
#define __MAPSERVICE_H

#include "upns_globals.h"
#include "services.pb.h"
#include "abstractlayerdatastreamprovider.h"
#include "layerdata.h"
#include "error.h"
//#include "mapservice.h"
//#include "map.h"

namespace upns
{

using MapVector = upnsVec< upnsSharedPointer<Map> >;
using LayerVector = upnsVec< upnsSharedPointer<Layer> >;

template<typename T>
bool upnsCheckResultVector( T result )
{
    return std::all_of(result.begin(), result.end(), [](typename T::value_type t){return upnsIsOk(t.second);});
}

/**
 * @brief The MapService class serves ability to read/write maps to/from a source without futher logic.
 * It can be seen as the abstraction of a file system. E.g. Versioning can be implemented on top of it.
 * E.g. 'remove' strictly deletes the physical representation of an entity without versioning it.
 * Concrete implementations for sources like network, filesystem, ... could exist, have this in mind when changing the class.
 * Layerdata Stream provider gives is part of this interface because the underlaying system must read/write layerdata as a stream.
 * To use layerdata as a user, a convenience class should be instanciated which gives easy access to the data in the layerspecific format
 * (e.g. you don't want to access parts of a stream but 'local regions' and 'areas').
 *
 * Locking:
 * Very important: Implementation should be thread safe, operations are atomic. Moreover it should be impossible
 * to open the same datasource multiple times, i.e. in diffrent processes.
 *
 * Maps/Layers use optimistic locking. If a write occurs and the system detects incompatible timestamps/versions, the store will fail.
 * It is very important to _not_ change the filds _id_, _version_ or _timestamp_ of the retrieved entities.
 * These are used by MapService internally.
 * Example:
 *   This happens, when the same map is read twice - two objects are returned by the system (no smartpointers to the same object!).
 *   Object one is changed and saved. By doing so, the MapService-Implementation will compare
 *   the objects version/timestamp with the old object (old object identified by id).
 *   If the versions/timestamps are equal, a new version/timestamp is generated and the object is stored/overwritten.
 *   For the second object, the comparison will fail due to an too old version/timestamp.
 *
 * Layerdata uses pessimistic locking. It allowes multiple reads and forbids simultaneous writes/readwrites.
 * The Versioning System does not allow multiple writes by definition. However with clever memory management it could happen,
 * that the physical representation of a state in history is acquired twice, e.g. a visualization wants to read data which
 * is currently overwritten by an in-memory algorithm.
 * The version system on top of the serialization must detect the locked layerdata and either:
 * 1. copy the (phyical) data for write or
 * 2. calculate the physical representation of the layer again from history
 * There are locking methods for the system to detect collisions.
 * - Data locked for read,  then read  access -> ok
 * - Data locked for read,  then write access -> copy layerdata
 * - Data locked for write, then read  access -> recalculate from history
 * - Data locked for write, then write access -> recalculate from history
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

    /// convenience ///
    upnsSharedPointer<Map> getMap( MapIdentifier mapId );
    StatusCode storeMap( upnsSharedPointer<Map> map );
    StatusCode removeMap(MapIdentifier mapId);
    ///////////////////

    //virtual upnsSharedPointer<Map> receiveNewMap(upnsString name) = 0;

    virtual bool canRead() = 0;
    virtual bool canWrite() = 0;

    virtual LockHandle lockLayerdataForRead(MapIdentifier mapId, LayerIdentifier layerId) = 0;
    virtual LockHandle lockLayerdataForWrite(MapIdentifier mapId, LayerIdentifier layerId) = 0;
    virtual void unlockLayerdataForWrite(LockHandle lockHandle) = 0;
    virtual void unlockLayerdataForRead(LockHandle lockHandle) = 0;

    virtual bool isLayerdataLockedForRead(MapIdentifier mapId, LayerIdentifier layerId) = 0;
    virtual bool isLayerdataLockedForWrite(MapIdentifier mapId, LayerIdentifier layerId) = 0;

    virtual upnsSharedPointer<AbstractLayerDataStreamProvider> getStreamProvider(MapIdentifier mapId, LayerIdentifier layerId) = 0;
};

}
#endif
