#ifndef __MAPMANAGER_H
#define __MAPMANAGER_H

#include "upns_globals.h"
#include "mapservice.h"
#include "yaml-cpp/yaml.h"
#include "layerdata.h"

/**
 * @brief The MapManager class makes maps an layer available to the client.
 * The class acts as a facade and hides complex actions needed to create/delete/change maps.
 * MapManager guarantees a consistent state of all maps.
 * It may handle caching.
 * Storage of data is done using the intenal MapService. MapService is another facade, which capsulates
 * physical storage, which could be e.g. Filesystem, Database or Network.
 *
 * MapManager can always read Maps. Writing is done by Operations. The Data of each layer can be queried
 * by using getLayerData which returns an abstract type. Depending on the type of layer that was queried,
 * the abstract type can be casted to are more useful class with custom ways to query layer data.
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

class MapManager
{
public:
    MapManager(const YAML::Node &config);
    ~MapManager();
    upnsVec<MapIdentifier> listMaps();
    MapVector getMaps(upnsVec<MapIdentifier> &mapIds);

    /// convenience ///
    upnsSharedPointer<Map> getMap( MapIdentifier mapId );
    ///////////////////

    bool canRead();
    bool canWrite();

    upnsSharedPointer<AbstractEntityData> getEntityData(MapIdentifier mapId, LayerIdentifier layerId, EntityIdentifier entityId);
    StatusCode doOperation(const OperationDescription &desc);

    /**
    * @brief getMapService is used to retrieve the internal mapservice
    * This Method exposes internal implementation and should not be used. (It is here for unit testing)
    * TODO: Read book about testing. What to do when a unit test is the only place a method is needed to be public?
    * TODO: The Test uses map service to write maps directly. Usually it would have to use mapmanager to do this.
    *       Unit test would get bigger then.
    * @return
    */
    MapService *getInternalMapService();

private:
    MapService *m_innerService;

    upnsSharedPointer<AbstractEntityData> wrapEntityOfType(LayerType type,
                                                         upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider);
    upnsSharedPointer<AbstractEntityData> wrapEntityOfType(upnsString layertypeName,
                                                         upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider);
};

}
#endif
