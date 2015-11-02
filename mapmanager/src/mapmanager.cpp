#include "mapmanager.h"
#include "upns.h"
#include "mapfileservice.h"
#include <string>
#include <algorithm>
#include <log4cplus/logger.h>

#define log_error(msg) log4cplus::Logger::getInstance("mapmanager").log(log4cplus::ERROR_LOG_LEVEL, msg)
#define log_warn(msg) log4cplus::Logger::getInstance("mapmanager").log(log4cplus::WARN_LOG_LEVEL, msg)
#define log_info(msg) log4cplus::Logger::getInstance("mapmanager").log(log4cplus::INFO_LOG_LEVEL, msg)

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

upnsVec<upnsPair<MapIdentifier, int> > MapManager::storeMaps(MapVector &maps)
{
    return m_innerService->storeMaps( maps );
}

upnsSharedPointer<Map> MapManager::createMap(upnsString name)
{
    return m_innerService->createMap( name );
}

MapResultsVector MapManager::removeMaps(upnsVec<MapIdentifier> &mapIds)
{
    return m_innerService->removeMaps( mapIds );
}

upnsSharedPointer<AbstractLayerDataStreamProvider> MapManager::getStreamProvider(MapIdentifier mapId, LayerIdentifier layerId)
{
    return m_innerService->getStreamProvider( mapId, layerId );
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

}
