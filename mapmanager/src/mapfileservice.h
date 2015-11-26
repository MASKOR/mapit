#ifndef __MAPFILESERVICE_H
#define __MAPFILESERVICE_H

#include "upns_globals.h"
#include "mapservice.h"
#include "yaml-cpp/yaml.h"
#include "abstractlayerdatastreamprovider.h"

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

private:
    leveldb::DB* m_db;

    upnsuint32 levelDbStatusToUpnsStatus(const leveldb::Status &levelDbStatus);
};

}
#endif
