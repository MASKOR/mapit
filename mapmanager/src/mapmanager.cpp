#include "mapmanager.h"
#include "upns.h"
#include "mapfileservice.h"
#include "filelayerdatastreamprovider.h"
#include <string>
#include <algorithm>
#include <log4cplus/logger.h>
#include <dlfcn.h>
#include "module.h"
#include "operationenvironment.h"

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

MapService *MapManager::getInternalMapService()
{
    return m_innerService;
}

upnsSharedPointer<AbstractEntityData> MapManager::getEntityData(MapIdentifier    mapId,
                                                                LayerIdentifier  layerId,
                                                                EntityIdentifier entityId)
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

    upnsSharedPointer<AbstractEntityData> layerData = wrapEntityOfType( layer->type(), m_innerService->getStreamProvider(mapId, layerId, entityId) ) ;
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

typedef ModuleInfo* (*GetModuleInfo)();

StatusCode MapManager::doOperation(const OperationDescription &desc)
{
    OperationEnvironment env(desc);
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
    std::stringstream filename;
    filename << "../operator_modules/" << desc.operatorname() << "/" << prefix << desc.operatorname() << debug << postfix;
    if(desc.operatorversion())
    {
        filename << "." << desc.operatorversion();
    }
    std::cout << filename.str() << std::endl;
    void* handle = dlopen(filename.str().c_str(), RTLD_NOW);
    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return UPNS_STATUS_ERR_MODULE_OPERATOR_NOT_FOUND;
    }
    GetModuleInfo getModInfo = (GetModuleInfo)dlsym(handle, "getModuleInfo");
    ModuleInfo* info = getModInfo();
    info->operate( &env );
    return UPNS_STATUS_OK;
}

upnsSharedPointer<AbstractEntityData> MapManager::wrapEntityOfType(LayerType type,
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

upnsSharedPointer<AbstractEntityData> MapManager::wrapEntityOfType(upnsString layertypeName,
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
