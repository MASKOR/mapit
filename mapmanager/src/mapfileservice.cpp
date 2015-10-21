#include "mapfileservice.h"
#include "upns.h"
#include "leveldb/db.h"
#include <assert.h>
#include <services.pb.h>
#include <stdlib.h>
#include <time.h>

#include <QDebug>

namespace upns
{

MapFileService::MapFileService(upnsString databaseFileName)
{
    leveldb::Options options;
    options.create_if_missing = true;
    leveldb::Status status = leveldb::DB::Open(options, databaseFileName, &m_db);
    assert(status.ok());

    srand (time(NULL));
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
            const MapIdentifier mi = *reinterpret_cast<const MapIdentifier*>(&(key.data()[key.size()-sizeof(MapIdentifier)]));
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
        MapIdentifier ident = *iter;
        key.append(std::string(reinterpret_cast<const char*>(&ident), sizeof(MapIdentifier)));
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

upnsVec<upnsPair<MapIdentifier, int> > upns::MapFileService::storeMaps(MapVector &maps)
{
    upnsVec<upnsPair<MapIdentifier, int> > ret;
    for(MapVector::const_iterator iter(maps.begin()) ; iter != maps.end() ; iter++)
    {
        std::string key("map!");
        MapIdentifier id = (*iter)->id();
        key.append(std::string(reinterpret_cast<const char*>(&id), sizeof(MapIdentifier)));
        leveldb::Status s = m_db->Put(leveldb::WriteOptions(), key, (*iter)->SerializeAsString());
        ret.push_back(upnsPair<MapIdentifier, int>(id, s.ok()));
    }
    return ret;
}

upnsSharedPointer<Map> upns::MapFileService::createMap(upnsString name)
{
    upnsSharedPointer<Map> newMap(new Map());
    newMap->set_name( name );
    newMap->set_id(static_cast<::google::protobuf::int64>(rand()));
    MapVector vec;
    vec.push_back(newMap);
    upnsVec<upnsPair<MapIdentifier, int> > res = storeMaps( vec );
    assert( res.at(0).second );
    return newMap;
}

}
