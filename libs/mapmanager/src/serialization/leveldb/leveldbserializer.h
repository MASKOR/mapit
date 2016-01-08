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

    virtual bool canRead();
    virtual bool canWrite();

    virtual upnsSharedPointer<Tree> getTree(const ObjectId &oid);
    virtual StatusCode storeTree(upnsSharedPointer<Tree> &obj);
    virtual StatusCode createTree(upnsSharedPointer<Tree> &obj);
    virtual StatusCode removeTree(const ObjectId &oid);

    virtual upnsSharedPointer<Entity> getEntity(const ObjectId oid);
    virtual StatusCode storeEntity(upnsSharedPointer<Entity> &obj);
    virtual StatusCode createEntity(upnsSharedPointer<Entity> &obj);
    virtual StatusCode removeEntity(const ObjectId &oid);

    virtual upnsSharedPointer<Commit> getCommit(const ObjectId &oid);
    virtual StatusCode storeCommit(upnsSharedPointer<Commit> &obj);
    virtual StatusCode createCommit(upnsSharedPointer<Commit> &obj);
    virtual StatusCode removeCommit(const ObjectId &oid);

    virtual upnsVec< ObjectId > listCheckoutIds();
    virtual upnsVec< upnsSharedPointer<Commit> > listCheckouts();
    virtual upnsSharedPointer<Commit> getCheckoutCommit(const ObjectId &oid);
    virtual StatusCode storeCheckoutCommit(upnsSharedPointer<Commit> &obj);
    virtual StatusCode createCheckoutCommit(upnsSharedPointer<Commit> &obj);
    virtual StatusCode removeCheckoutCommit(const ObjectId &oid);

    virtual upnsVec< upnsSharedPointer<Branch> > listBranches();
    virtual upnsSharedPointer<Branch> getBranch(const ObjectId &oid);
    virtual StatusCode storeBranch(upnsSharedPointer<Branch> &obj);
    virtual StatusCode createBranch(upnsSharedPointer<Branch> &obj);
    virtual StatusCode removeBranch(const ObjectId &oid);

    virtual upnsSharedPointer<AbstractEntityDataStreamProvider> getStreamProvider(const ObjectId &entityId);

    /**
     * @brief cleanUp Collects Grabage. Orphan Objects, not reachable by any branch are removed.
     * @return
     */
    virtual StatusCode cleanUp();

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

    StatusCode getObject(const std::string &key, std::string &value);
    StatusCode storeObject(const std::string &key, std::string &value);

    template <typename T>
    upnsSharedPointer<T> getObject(const std::string &key);

    template <typename T>
    StatusCode storeObject(const std::string &key, upnsSharedPointer<T> value);

    template <typename T>
    StatusCode createObject(const std::string &key, upnsSharedPointer<T> value);
};

}
#endif
