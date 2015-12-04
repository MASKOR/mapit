#include "mapservice.h"
#include "upns.h"

#include <dlfcn.h>

namespace upns
{

MapService::MapService(MapSerializer *serializer)
    :m_innerSerializer(serializer)
{
}

MapService::~MapService()
{
    assert(m_innerSerializer);
    if(m_innerSerializer)
    {
        delete m_innerSerializer;
    }
}

bool MapService::canRead()
{
    return m_innerSerializer->canRead();
}

bool MapService::canWrite()
{
    return m_innerSerializer->canWrite();
}

upnsVec<MapIdentifier> MapService::listMaps()
{
    return m_innerSerializer->listMaps();
}

MapVector MapService::getMaps(upnsVec<MapIdentifier> &mapIds)
{
    return m_innerSerializer->getMaps(mapIds);
}

MapResultsVector MapService::storeMaps(MapVector &maps)
{
    return m_innerSerializer->storeMaps(maps);
}

upnsSharedPointer<Map> MapService::createMap(upnsString name)
{
    return m_innerSerializer->createMap(name);
}

MapResultsVector MapService::removeMaps(upnsVec<MapIdentifier> &mapIds)
{
    return m_innerSerializer->removeMaps(mapIds);
}

upnsSharedPointer<upns::Map> MapService::getMap(upns::MapIdentifier mapId)
{
    upnsVec<MapIdentifier> mapIds;
    mapIds.push_back(mapId);
    MapVector maps = getMaps( mapIds );
    assert(maps.size() == 1);
    return maps.at(0);
}

StatusCode MapService::storeMap(upnsSharedPointer<Map> map)
{
    MapVector maps;
    maps.push_back(map);
    MapResultsVector res = storeMaps( maps );
    assert(res.size() == 1);
    return res.at(0).second;
}

StatusCode MapService::removeMap(MapIdentifier mapId)
{
    upnsVec<MapIdentifier> mapIds;
    mapIds.push_back(mapId);
    MapResultsVector res = removeMaps( mapIds );
    assert(res.size() == 1);
    return res.at(0).second;
}

LockHandle MapService::lockLayerdataForRead(MapIdentifier mapId, LayerIdentifier layerId)
{
    //TODO
    return 0;
}

LockHandle MapService::lockLayerdataForWrite(MapIdentifier mapId, LayerIdentifier layerId)
{
    //TODO
    return 0;
}

void MapService::unlockLayerdataForWrite(LockHandle lockHandle)
{
    //TODO
}

void MapService::unlockLayerdataForRead(LockHandle lockHandle)
{
    //TODO
}

bool MapService::isLayerdataLockedForRead(MapIdentifier mapId, LayerIdentifier layerId)
{
    //TODO
    return false;
}

bool MapService::isLayerdataLockedForWrite(MapIdentifier mapId, LayerIdentifier layerId)
{
    //TODO
    return false;
}

upnsSharedPointer<AbstractEntityData> MapService::getEntityData(MapIdentifier    mapId,
                                                                LayerIdentifier  layerId,
                                                                EntityIdentifier entityId)
{
    upnsVec<MapIdentifier> mapIds;
    mapIds.push_back( mapId );
    MapVector maps = m_innerSerializer->getMaps( mapIds );
    upnsSharedPointer<Map> map(maps.at(0));
    const Layer *layer = NULL;
    for(int i=0 ; i<map->layers_size() ; ++i)
    {
        const Layer &curLayer = map->layers(i);
        if(curLayer.id() == layerId)
        {
            layer = &curLayer;
            break;
        }
    }
    assert( layer );

    upnsSharedPointer<AbstractEntityData> layerData = wrapEntityOfType( layer->type(), m_innerSerializer->getStreamProvider(mapId, layerId, entityId) ) ;
    return layerData;
}

upnsSharedPointer<AbstractEntityData> MapService::wrapEntityOfType(LayerType type,
                                                                   upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
{
    upnsString layerName;
    switch(type)
    {
    case POINTCLOUD2:
    {
        layerName = "layertype_pointcloud2";
        break;
    }
    default:
        log_error("Unknown layertype: " + std::to_string(type));
        return upnsSharedPointer<AbstractEntityData>(NULL);
    }
    return wrapEntityOfType( layerName, streamProvider );
}

upnsSharedPointer<AbstractEntityData> MapService::wrapEntityOfType(upnsString layertypeName,
                                                                  upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
{
    upnsSharedPointer<AbstractEntityData> ret;
#ifndef NDEBUG
    upnsString debug = "d";
#else
    upnsString debug = "";
#endif

#ifdef _WIN32
    upnsString prefix = "";
    upnsString postfix = ".dll";
#else
    upnsString prefix = "lib";
    upnsString postfix = ".so";
#endif
    void* handle = dlopen((upnsString("../layertypes/") + prefix + layertypeName + debug + postfix).c_str(), RTLD_NOW);
    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return upnsSharedPointer<AbstractEntityData>(NULL);
    }
    WrapLayerTypeFunc wrap = (WrapLayerTypeFunc)dlsym(handle, "createEntityData");
    return upnsSharedPointer<AbstractEntityData>( wrap( streamProvider ) );
}

}
