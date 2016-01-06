#ifndef __LEVELDBSERIALIZER_H
#define __LEVELDBSERIALIZER_H

#include "upns_globals.h"
#include "../abstractmapserializerNEW.h"
#include "yaml-cpp/yaml.h"
#include "abstractentitydatastreamprovider.h"

class QLockFile;
namespace leveldb {
    class DB;
    class Status;
}

namespace upns
{

/**
 * @brief The LevelDBSerializer class stores all data using leveldb.
 *
 */

class LevelDBSerializer : public AbstractMapSerializer
{
public:
    LevelDBSerializer(const YAML::Node &config);
    virtual ~LevelDBSerializer();

    bool canRead() = 0;
    bool canWrite() = 0;

    upnsSharedPointer<Tree> getTree(const ObjectId &oid) = 0;
    StatusCode storeTree(upnsSharedPointer<Tree> &tree) = 0;
    StatusCode createTree(upnsSharedPointer<Tree> &tree) = 0;
    StatusCode removeTree(const ObjectId &oid) = 0;

    upnsSharedPointer<Entity> getEntity(const ObjectId oid) = 0;
    StatusCode storeEntity(upnsSharedPointer<Entity> &tree) = 0;
    StatusCode createEntity(upnsSharedPointer<Entity> &entity) = 0;
    StatusCode removeEntity(const ObjectId &oid) = 0;

private:
    leveldb::DB* m_db;

    StatusCode levelDbStatusToUpnsStatus(const leveldb::Status &levelDbStatus);
    QLockFile *m_lockFile;

    std::string mapKey(MapIdentifier mapId) const;
    std::string entityKey(MapIdentifier mapId, LayerIdentifier layerId, EntityIdentifier entityId) const;    
};

}
#endif
