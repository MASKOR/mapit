#ifndef __MAPFILESERVICE_H
#define __MAPFILESERVICE_H

#include "upns_globals.h"
#include "mapservice.h"
#include "yaml-cpp/yaml.h"
#include "abstractlayerdatastreamprovider.h"

class QLockFile;
namespace leveldb {
    class DB;
    class Status;
}

namespace upns
{

/**
 * @brief The MapFileService class stores all data using leveldb.
 *
 * Database Schema:
 * leveldb saves key -> value pairs. MapFileService saves Protobuf entities as values. Entitydata ist stored as is (binary).
 * Entries:
 *  Map: Key: map!<mapId>
 *  Entity: Key: entity!<mapId>!<layerId>!<entityId>
 */

class MapFileService : public MapService
{
public:
    MapFileService(const YAML::Node &config);
    ~MapFileService();
    upnsVec<MapIdentifier> listMaps();
    MapVector getMaps(upnsVec<MapIdentifier> &mapIds);
    MapResultsVector storeMaps( MapVector &maps );
    upnsSharedPointer<Map> createMap(upnsString name);
    MapResultsVector removeMaps(upnsVec<MapIdentifier> &mapIds);

    upnsSharedPointer<AbstractEntityDataStreamProvider> getStreamProvider(MapIdentifier    mapId,
                                                                         LayerIdentifier  layerId,
                                                                         EntityIdentifier entityId);

    bool canRead();
    bool canWrite();

    LockHandle lockLayerdataForRead(MapIdentifier mapId, LayerIdentifier layerId);
    LockHandle lockLayerdataForWrite(MapIdentifier mapId, LayerIdentifier layerId);
    void unlockLayerdataForWrite(LockHandle lockHandle);
    void unlockLayerdataForRead(LockHandle lockHandle);
    bool isLayerdataLockedForRead(MapIdentifier mapId, LayerIdentifier layerId);
    bool isLayerdataLockedForWrite(MapIdentifier mapId, LayerIdentifier layerId);
private:
    leveldb::DB* m_db;

    StatusCode levelDbStatusToUpnsStatus(const leveldb::Status &levelDbStatus);
    QLockFile *m_lockFile;

    std::string mapKey(MapIdentifier mapId) const;
    std::string entityKey(MapIdentifier mapId, LayerIdentifier layerId, EntityIdentifier entityId) const;
};

}
#endif
