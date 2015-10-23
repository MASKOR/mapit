#ifndef __MAPMANAGER_H
#define __MAPMANAGER_H

#include "upns_globals.h"
#include "mapservice.h"
#include "yaml-cpp/yaml.h"

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
 * * * aggregation: Objects have getters for easy-to-use access of aggregated objects as members.
 * * * * Indeed not the whole graph of objects is deserialized at once.
 * * * * objects live in a context / are associated to "map manager" which is able to resolve aggregation.
 * Every object can serialize/deserialize to/from a given source. Examples for sources: network, harddisk, memory (e.g. on undo-command).
 * Objects have no
 *
 * MapGraph -> Map:                 lazy
 * Map      -> Layer:               hard
 * Layer    -> LayerData/Region:    lazy
 * Layer    -> history:             lazy
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

    upnsSharedPointer<Map> doOperation(upnsString config);
private:
    MapService *m_innerService;
};

}
#endif
