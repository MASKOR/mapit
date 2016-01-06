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

    virtual bool canRead() = 0;
    virtual bool canWrite() = 0;

    virtual upnsSharedPointer<Tree> getTree(const ObjectId &oid) = 0;
    virtual StatusCode storeTree(upnsSharedPointer<Tree> &tree) = 0;
    virtual StatusCode createTree(upnsSharedPointer<Tree> &tree) = 0;
    virtual StatusCode removeTree(const ObjectId &oid) = 0;

    virtual upnsSharedPointer<Entity> getEntity(const ObjectId oid) = 0;
    virtual StatusCode storeEntity(upnsSharedPointer<Entity> &entity) = 0;
    virtual StatusCode createEntity(upnsSharedPointer<Entity> &entity) = 0;
    virtual StatusCode removeEntity(const ObjectId &oid) = 0;

    virtual upnsSharedPointer<Commit> getCommit(const ObjectId &oid) = 0;
    virtual StatusCode storeCommit(upnsSharedPointer<Commit> &obj) = 0;
    virtual StatusCode createCommit(upnsSharedPointer<Commit> &obj) = 0;
    virtual StatusCode removeCommit(const ObjectId &oid) = 0;

    virtual upnsVec< upnsSharedPointer<Commit> > listCheckouts() = 0;
    virtual upnsSharedPointer<Commit> getCheckoutCommit(const ObjectId &oid) = 0;
    virtual StatusCode storeCheckoutCommit(upnsSharedPointer<Commit> &obj) = 0;
    virtual StatusCode createCheckoutCommit(upnsSharedPointer<Commit> &obj) = 0;
    virtual StatusCode removeCheckoutCommit(const ObjectId &oid) = 0;

    virtual upnsVec< upnsSharedPointer<Branch> > listBranches() = 0;
    virtual upnsSharedPointer<Branch> getBranch(const ObjectId &oid) = 0;
    virtual StatusCode storeBranch(upnsSharedPointer<Branch> &obj) = 0;
    virtual StatusCode createBranch(upnsSharedPointer<Branch> &obj) = 0;
    virtual StatusCode removeBranch(const ObjectId &oid) = 0;

    virtual upnsSharedPointer<AbstractEntityDataStreamProvider> getStreamProvider(const ObjectId &entityId) = 0;

    /**
     * @brief cleanUp Collects Grabage. Orphan Objects, not reachable by any branch are removed.
     * @return
     */
    virtual StatusCode cleanUp() = 0;

private:
    leveldb::DB* m_db;

    StatusCode levelDbStatusToUpnsStatus(const leveldb::Status &levelDbStatus);
    QLockFile *m_lockFile;

    std::string keyOfTree(const ObjectId &oid) const;
    std::string keyOfEntity(const ObjectId &oid) const;
    std::string keyOfEntityData(const ObjectId &oid) const;
    std::string keyOfCommit(const ObjectId &oid) const;
    std::string keyOfCheckoutCommit(const ObjectId &oid) const;
    std::string keyOfBranch(const ObjectId &oid) const;
};

}
#endif
