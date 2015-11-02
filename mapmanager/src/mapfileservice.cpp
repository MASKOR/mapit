#include "mapfileservice.h"
#include "upns.h"
#include "util.h"
#include "filelayerdatastreamprovider.h"
#include "leveldb/db.h"
#include <assert.h>
#include <services.pb.h>
#include <stdlib.h>
#include <time.h>
#include <log4cplus/logger.h>

#include <QDebug>
#include <QDateTime>

#define log_error(msg) log4cplus::Logger::getInstance("mapfileservice").log(log4cplus::ERROR_LOG_LEVEL, msg)
#define log_warn(msg) log4cplus::Logger::getInstance("mapfileservice").log(log4cplus::WARN_LOG_LEVEL, msg)
#define log_info(msg) log4cplus::Logger::getInstance("mapfileservice").log(log4cplus::INFO_LOG_LEVEL, msg)

namespace upns
{

MapFileService::MapFileService(const YAML::Node &config)
{
    std::string databaseName;
    if(config["filename"])
    {
        databaseName = config["filename"].as<std::string>();
    }
    else
    {
        databaseName = "maps.db";
    }
    log_info("opening database " + databaseName);
    leveldb::Options options;
    options.create_if_missing = true;
    leveldb::Status status = leveldb::DB::Open(options, databaseName, &m_db);
    assert(status.ok());
}

MapFileService::~MapFileService()
{
    delete m_db;
}

upnsVec<MapIdentifier> upns::MapFileService::listMaps()
{
    upnsVec<MapIdentifier> ret;
    leveldb::Iterator* it = m_db->NewIterator(leveldb::ReadOptions());
    for (it->SeekToFirst(); it->Valid(); it->Next()) {
        const leveldb::Slice &key = it->key();
        if(key.starts_with("map!"))
        {
            const MapIdentifier mi = sliceEndToId( key );
            ret.push_back( mi );
        }
    }
    assert(it->status().ok());  // Check for any errors found during the scan
    delete it;
    return ret;
}

MapVector upns::MapFileService::getMaps(upnsVec<MapIdentifier> &mapIds)
{
    MapVector ret;

    Map map;
    for(upnsVec<MapIdentifier>::const_iterator iter(mapIds.begin()) ; iter != mapIds.end() ; iter++)
    {
        std::string key("map!");
        key.append( idToString( *iter ));
        std::string value;
        leveldb::Status s = m_db->Get(leveldb::ReadOptions(), key, &value);
        assert(s.ok());
        if(! map.ParseFromString( value ))
        {
            std::cout << "could not parse: " << key;
            continue;
        }
        ret.push_back( upnsSharedPointer<Map>( new Map(map) ) );
    }
    return ret;
}

MapResultsVector upns::MapFileService::storeMaps(MapVector &maps)
{
    upnsVec<upnsPair<MapIdentifier, int> > ret;
    for(MapVector::const_iterator iter(maps.begin()) ; iter != maps.end() ; iter++)
    {
        std::string key("map!");
        const MapIdentifier &id = (*iter)->id();
        key.append( idToString( id ));
        (*iter)->set_lastchange(QDateTime::currentDateTime().toMSecsSinceEpoch());
        for(int i=0; i < (*iter)->layers_size() ; ++i)
        {
            Layer *layer = (*iter)->mutable_layers(i);
            if(layer->id() == 0)
            {
                layer->set_id(generateId());
            }
        }
        leveldb::Status s = m_db->Put(leveldb::WriteOptions(), key, (*iter)->SerializeAsString());
        ret.push_back(upnsPair<MapIdentifier, int>(id, s.ok()));
    }
    return ret;
}

upnsSharedPointer<Map> upns::MapFileService::createMap(upnsString name)
{
    upnsSharedPointer<Map> newMap(new Map());
    newMap->set_name( escapeName( name ) );
    newMap->set_id( generateId() );
    MapVector vec;
    vec.push_back(newMap);
    upnsVec<upnsPair<MapIdentifier, int> > res = storeMaps( vec );
    return res.at(0).second?newMap:upnsSharedPointer<Map>(NULL);
}

MapResultsVector MapFileService::removeMaps(upnsVec<MapIdentifier> &mapIds)
{
    upnsVec<upnsPair<MapIdentifier, int> > ret;
    for(upnsVec<MapIdentifier>::const_iterator iter(mapIds.begin()) ; iter != mapIds.end() ; iter++)
    {
        std::string key("map!");
        key.append( idToString( *iter ));
        leveldb::Status s = m_db->Delete(leveldb::WriteOptions(), key);
        ret.push_back(upnsPair<MapIdentifier, int>(*iter, s.ok()));
    }
    return ret;
}

upnsSharedPointer<AbstractLayerDataStreamProvider> MapFileService::getStreamProvider(MapIdentifier mapId, LayerIdentifier layerId)
{
    std::string key = idToString(mapId);
    key.append( idToString(layerId) );
    return upnsSharedPointer<AbstractLayerDataStreamProvider>( new FileLayerDataStreamProvider(m_db, leveldb::Slice(key)));
}

bool MapFileService::canRead()
{
    return true;
}

bool MapFileService::canWrite()
{
    return true;
}

}
