#include "mapmanager.h"
#include "upns.h"
#include "mapleveldb/mapleveldbserializer.h"
#include "mapleveldb/leveldbentitydatastreamprovider.h"
#include <string>
#include <algorithm>
#include <log4cplus/logger.h>
#include <dlfcn.h>
#include "module.h"
#include "operationenvironmentimpl.h"

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
            MapSerializer *mser = NULL;
            if(mapsrcnam == "mapfileservice")
            {
                mser = new MapLeveldbSerializer(mapsource);
            } else {
                log_error("mapsource '" + mapsrcnam + "' was not found.");
            }
            if(mser)
            {
                m_innerService = new MapService(mser);
            }
        } else {
            log_error("'mapsource' has no 'name' in config");
        }
    } else {
        log_error("Key 'mapsource' not given in config");
    }
    assert(m_innerService);
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

upnsSharedPointer<Map> MapManager::getMap(MapIdentifier mapId)
{
    upnsVec<MapIdentifier> mapIds;
    mapIds.push_back(mapId);
    MapVector mapvec(m_innerService->getMaps( mapIds ));
    if(mapvec.empty())
    {
        return upnsSharedPointer<Map>(NULL);
    }
    else
    {
        return mapvec.at(0);
    }
}

MapService *MapManager::getInternalMapService()
{
    return m_innerService;
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

OperationResult MapManager::doOperation(const OperationDescription &desc)
{
    OperationEnvironmentImpl env(desc);
    env.setMapManager( this );
    env.setMapService( this->m_innerService );
#ifndef NDEBUG
    upnsString debug = DEBUG_POSTFIX;
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
    void* handle = dlopen(filename.str().c_str(), RTLD_NOW);
    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return OperationResult(UPNS_STATUS_ERR_MODULE_OPERATOR_NOT_FOUND, OperationDescription());
    }
    GetModuleInfo getModInfo = (GetModuleInfo)dlsym(handle, "getModuleInfo");
    ModuleInfo* info = getModInfo();
    StatusCode result = info->operate( &env );
    if(!upnsIsOk(result))
    {
        std::stringstream strm;
        strm << "operator '" << desc.operatorname() << "' reported an error. (code:" << result << ")";
        log_error(strm.str());
    }
    return OperationResult(result, env.outputDescription());
}

}
