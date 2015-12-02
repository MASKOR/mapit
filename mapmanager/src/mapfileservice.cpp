#include "mapfileservice.h"
#include "upns.h"
#include "util.h"
#include "filelayerdatastreamprovider.h"
#include "leveldb/db.h"
#include <assert.h>
#include <services.pb.h>
#include <stdlib.h>
#include <time.h>
#include "error.h"

#include <QDebug>
#include <QDateTime>
#include <QFileInfo>
#include <QLockFile>

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

    // check lockfile, database must only be opened once.
    // Note: leveldb seems to have locking on its own and does not allow simultaneous access at all
    QString databaseNameQStr = QString::fromStdString(databaseName);
    m_lockFile = new QLockFile(QFileInfo(databaseNameQStr).absolutePath() + "." + QFileInfo(databaseNameQStr).fileName() + ".lock");
    if(!m_lockFile->tryLock())
    {
        qint64 pid;
        QString hostname;
        QString appname;
        m_lockFile->getLockInfo( &pid, &hostname, &appname);
        log_error(std::string("database already open. Indicated by lockfile ") + QFileInfo(databaseNameQStr).canonicalFilePath().toStdString()
                  + "\n PID: " + QString::number(pid).toStdString()
                  + "\n Hostname: " + hostname.toStdString()
                  + "\n Application: " + appname.toStdString());
    }

    leveldb::Options options;
    options.create_if_missing = true;
    leveldb::Status status = leveldb::DB::Open(options, databaseName, &m_db);
    assert(status.ok());
}

MapFileService::~MapFileService()
{
    // destruction of m_lockFile will unlock the database if previously acquired.
    delete m_db;
    delete m_lockFile;
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
            log_error("could not parse: " + key);
            continue;
        }
        ret.push_back( upnsSharedPointer<Map>( new Map(map) ) );
    }
    return ret;
}

MapResultsVector upns::MapFileService::storeMaps(MapVector &maps)
{
    MapResultsVector ret;
    for(MapVector::const_iterator iter(maps.begin()) ; iter != maps.end() ; iter++)
    {
        std::string key("map!");
        const MapIdentifier &id = (*iter)->id();
        key.append( idToString( id ));

        // Get the previously stored map to check for correct version (optimistic locking)
        // Note: leveldb has no transactions. It is not guaranteed, that stuff is done between the next "Get" and "Put".
        // A guarantee can be given, if only one thread is used to access the database.
        std::string str;
        leveldb::Status s = m_db->Get(leveldb::ReadOptions(), key, &str);
        if(s.ok())
        {
            Map prevMap;
            if(!prevMap.ParseFromString( str ))
            {
                log_error("could not parse: " + key);
                continue;
            }
            if(prevMap.lastchange() != (*iter)->lastchange())
            {
                ret.push_back(upnsPair<MapIdentifier, int>(id, UPNS_STATUS_ERR_DB_OPTIMISTIC_LOCKING));
                continue;
            }
        }
        else if( !s.IsNotFound() )
        {
            log_error("error while checking entity: " + key);
        }

        (*iter)->set_lastchange(QDateTime::currentDateTime().toMSecsSinceEpoch());
        for(int i=0; i < (*iter)->layers_size() ; ++i)
        {
            Layer *layer = (*iter)->mutable_layers(i);
            if(layer->id() == 0)
            {
                layer->set_id(generateId());
            }
        }
        s = m_db->Put(leveldb::WriteOptions(), key, (*iter)->SerializeAsString());
        ret.push_back(StatusPair(id, levelDbStatusToUpnsStatus(s)));
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
    upnsVec<upnsPair<MapIdentifier, StatusCode> > res = storeMaps( vec );
    return upnsIsOk(res.at(0).second)?newMap:upnsSharedPointer<Map>(NULL);
}

MapResultsVector MapFileService::removeMaps(upnsVec<MapIdentifier> &mapIds)
{
    upnsVec<upnsPair<MapIdentifier, StatusCode> > ret;
    for(upnsVec<MapIdentifier>::const_iterator iter(mapIds.begin()) ; iter != mapIds.end() ; iter++)
    {
        std::string key("map!");
        std::string mapIdStr(idToString( *iter ));
        key.append( mapIdStr );

        // Get Map and all associated layers
        std::string value;
        leveldb::Status sread = m_db->Get(leveldb::ReadOptions(), key, &value);

        Map map;
        if(!sread.ok() || !map.ParseFromString( value ))
        {
            ret.push_back(StatusPair(*iter, UPNS_STATUS_ERR_DB_PARSE_MAP));
            continue;
        }
        // Delete layers
        bool layerDeletionSuccess = true;
        for(int i=0; i < map.layers_size() ; ++i)
        {
            Layer *layer = map.mutable_layers(i);
            if(layer->id() == 0)
            {
                layerDeletionSuccess = false;
                continue;
            }
            std::string key(mapIdStr);
            key.append( idToString(layer->id()) );
            leveldb::Status slayer = m_db->Delete(leveldb::WriteOptions(), key);
            layerDeletionSuccess &= slayer.ok();
        }
        if(!layerDeletionSuccess)
        {
            ret.push_back(upnsPair<MapIdentifier, int>(*iter, UPNS_STATUS_ERR_DB_DELETE_LAYER_FROM_MAP));
            continue;
        }

        // Finally delete map only if everything went ok until here
        leveldb::Status sdel = m_db->Delete(leveldb::WriteOptions(), key);
        ret.push_back(StatusPair(*iter, levelDbStatusToUpnsStatus( sdel )));
    }
    return ret;
}

upnsSharedPointer<AbstractLayerDataStreamProvider> MapFileService::getStreamProvider(MapIdentifier mapId, LayerIdentifier layerId)
{
    std::string key = idToString(mapId);
    key.append( idToString(layerId) );
    return upnsSharedPointer<AbstractLayerDataStreamProvider>( new FileLayerDataStreamProvider(m_db, key));
}

bool MapFileService::canRead()
{
    return true;
}

bool MapFileService::canWrite()
{
    return true;
}

LockHandle MapFileService::lockLayerdataForRead(MapIdentifier mapId, LayerIdentifier layerId)
{
    //Note: TODO: Locking is not possible with leveldb.
    // Leveldb can only be opened by one process.
    // Locking could be needed to handle mutliple threads.
    // Does it make sense here?
    // - member m_lock should be used. No database entries
}

LockHandle MapFileService::lockLayerdataForWrite(MapIdentifier mapId, LayerIdentifier layerId)
{

}

void MapFileService::unlockLayerdataForWrite(LockHandle lockHandle)
{

}

void MapFileService::unlockLayerdataForRead(LockHandle lockHandle)
{

}

bool MapFileService::isLayerdataLockedForRead(MapIdentifier mapId, LayerIdentifier layerId)
{

}

bool MapFileService::isLayerdataLockedForWrite(MapIdentifier mapId, LayerIdentifier layerId)
{

}

StatusCode MapFileService::levelDbStatusToUpnsStatus(const leveldb::Status &levelDbStatus)
{
    if(levelDbStatus.ok()) {
        return UPNS_STATUS_OK;
    } else if(levelDbStatus.IsNotFound()) {
        return UPNS_STATUS_ERR_DB_NOT_FOUND;
    } else if(levelDbStatus.IsCorruption()) {
        return UPNS_STATUS_ERR_DB_CORRUPTION;
    } else if(levelDbStatus.IsIOError()) {
        return UPNS_STATUS_ERR_DB_IO_ERROR;
    } else {
        // might be...
        // UPNS_STATUS_ERR_DB_NOT_SUPPORTED;
        // UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
        return UPNS_STATUS_ERR_DB_UNKNOWN;
    }
}

}
