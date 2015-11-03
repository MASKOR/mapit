#ifndef __MAPMANAGER_H
#define __MAPMANAGER_H

#include "upns_globals.h"
#include "mapservice.h"
#include "yaml-cpp/yaml.h"
#include "layerdata.h"

/**
 * @brief The MapGraph class contains maps and their versions logically.
 * It does not do caching or calculation.
 *
 * Serialization:
 * Goals:
 * * Solution for Classes/Objects and binarydata for layers.
 * * Don't try to solve serialization problems on a too abstract level and stick strictly to requirements.
 * * * multiple versions of the same object may exist. Developer has to prevent/coupe with that individually.
 * * * objects are not managed and do not serialize/deserialize automatically.
 * * * aggregation: Objects have getters for easy-to-use access of aggregated objects as members only sometimes (non lazy).
 * * * * Not the whole graph of objects is deserialized at once.
 * Every object can serialize/deserialize to/from a given source. Examples for sources: network, harddisk, memory (e.g. on undo-command).
 *
 * MapGraph -> Map:                 lazy  (listMaps -> getMap(s))
 * Map      -> Layer:               hard  (Map::layers)
 * Layer    -> LayerData/Region:    lazy  (getLayerdata)
 * Layer    -> history:             lazy  (not yet discussed, TODO)
 *
 * doOperation: Map + Layer + operationName + params inside config?
 * doOperation(/*upnsSharedPointer<Map> targetMap, upnsSharedPointer<Layer> targetLayer, upnsString &operationName,* / upnsString config);
 */

namespace upns
{

class MapManager : public MapService
{
public:
    MapManager(const YAML::Node &config);
    ~MapManager();
    upnsVec<MapIdentifier> listMaps();
    MapVector getMaps(upnsVec<MapIdentifier> &mapIds);
    MapResultsVector storeMaps( MapVector &maps );
    upnsSharedPointer<Map> createMap(upnsString name);
    MapResultsVector removeMaps(upnsVec<MapIdentifier> &mapIds);

    /// convenience ///
    upnsSharedPointer<Map> getMap( MapIdentifier mapId );
    int storeMap( upnsSharedPointer<Map> map );
    int removeMap(MapIdentifier mapId);
    ///////////////////

    bool canRead();
    bool canWrite();

    upnsSharedPointer<AbstractLayerData> getLayerData(MapIdentifier mapId, LayerIdentifier layerId);
    upnsSharedPointer<Map> doOperation(upnsString config);

    upnsSharedPointer<AbstractLayerDataStreamProvider> getStreamProvider(MapIdentifier mapId, LayerIdentifier layerId);

private:
    MapService *m_innerService;

    upnsSharedPointer<AbstractLayerData> wrapLayerOfType(LayerType type, upnsSharedPointer<AbstractLayerDataStreamProvider> streamProvider);
    upnsSharedPointer<AbstractLayerData> wrapLayerOfType(upnsString layertypeName, upnsSharedPointer<AbstractLayerDataStreamProvider> streamProvider);
};

}
#endif
