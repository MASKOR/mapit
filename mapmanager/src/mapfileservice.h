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

    upnsSharedPointer<AbstractLayerDataStreamProvider> getStreamProvider(MapIdentifier mapId, LayerIdentifier layerId);

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
};

}
#endif
