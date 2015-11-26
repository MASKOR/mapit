#include "mapmanager.h"
#include "upns.h"
#include "mapfileservice.h"
#include "filelayerdatastreamprovider.h"
#include <string>
#include <algorithm>
#include <log4cplus/logger.h>
#include <dlfcn.h>

namespace upns
{

MapManager::MapManager(const YAML::Node &config)
    :m_innerService(NULL)
{
    if(const YAML::Node mapsource = config["mapsource"])
    {
        if(const YAML::Node mapsourceName = mapsource["name"])
        {
            std::string mapsrcnam = mapsourceName.as<std::string>();
            std::transform(mapsrcnam.begin(), mapsrcnam.end(), mapsrcnam.begin(), ::tolower);
            if(mapsrcnam == "mapfileservice")
            {
                m_innerService = new MapFileService(mapsource);
            }
        } else {
            log_error("'mapsource' has no 'name' in config");
        }
    } else {
        log_error("Key 'mapsource' not given in config");
    }
}

MapManager::~MapManager()
{
    if(m_innerService)
    {
        delete m_innerService;
    }
}

upnsVec<MapIdentifier> MapManager::listMaps()
{
    return m_innerService->listMaps();
}

MapVector MapManager::getMaps(upnsVec<MapIdentifier> &mapIds)
{
    return m_innerService->getMaps( mapIds );
}

//MapResultsVector MapManager::storeMaps(MapVector &maps)
//{
//    return m_innerService->storeMaps( maps );
//}

//upnsSharedPointer<Map> MapManager::createMap(upnsString name)
//{
//    return m_innerService->createMap( name );
//}

//MapResultsVector MapManager::removeMaps(upnsVec<MapIdentifier> &mapIds)
//{
//    return m_innerService->removeMaps( mapIds );
//}

upnsSharedPointer<upns::Map> MapManager::getMap(upns::MapIdentifier mapId)
{
    upnsVec<MapIdentifier> mapIds;
    mapIds.push_back(mapId);
    MapVector maps = getMaps( mapIds );
    assert(maps.size() == 1);
    return maps.at(0);
}

//int MapManager::storeMap(upnsSharedPointer<Map> map)
//{
//    MapVector maps;
//    maps.push_back(map);
//    MapResultsVector res = storeMaps( maps );
//    assert(res.size() == 1);
//    return res.at(0).second;
//}

//int MapManager::removeMap(MapIdentifier mapId)
//{
//    upnsVec<MapIdentifier> mapIds;
//    mapIds.push_back(mapId);
//    MapResultsVector res = removeMaps( mapIds );
//    assert(res.size() == 1);
//    return res.at(0).second;
//}

upnsSharedPointer<AbstractLayerDataStreamProvider> MapManager::getStreamProvider(MapIdentifier mapId, LayerIdentifier layerId)
{
    return m_innerService->getStreamProvider( mapId, layerId );
}

upnsSharedPointer<AbstractLayerData> MapManager::getLayerData(MapIdentifier mapId, LayerIdentifier layerId)
{
    upnsVec<MapIdentifier> mapIds;
    mapIds.push_back( mapId );
    MapVector maps = m_innerService->getMaps( mapIds );
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

    upnsSharedPointer<AbstractLayerData> layerData = wrapLayerOfType( layer->type(), getStreamProvider(mapId, layerId) ) ;
    return layerData;
}

bool MapManager::canRead()
{
    return m_innerService->canRead();
}

bool MapManager::canWrite()
{
    return m_innerService->canWrite();
}

upnsSharedPointer<Map> MapManager::doOperation(upnsString config)
{

}

upnsSharedPointer<AbstractLayerData> MapManager::wrapLayerOfType(LayerType type, upnsSharedPointer<AbstractLayerDataStreamProvider> streamProvider)
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
        return upnsSharedPointer<AbstractLayerData>(NULL);
    }
    return wrapLayerOfType( layerName, streamProvider );
}

upnsSharedPointer<AbstractLayerData> MapManager::wrapLayerOfType(upnsString layertypeName, upnsSharedPointer<AbstractLayerDataStreamProvider> streamProvider)
{
    upnsSharedPointer<AbstractLayerData> ret;
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
        return upnsSharedPointer<AbstractLayerData>(NULL);
    }
    WrapLayerTypeFunc wrap = (WrapLayerTypeFunc)dlsym(handle, "createLayerData");
    return upnsSharedPointer<AbstractLayerData>( wrap( streamProvider ) );
}

}
