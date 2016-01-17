#ifndef LEVELDBSERIALIZER_H
#define LEVELDBSERIALIZER_H

#include "upns_globals.h"
#include "serialization/abstractmapserializer.h"
#include "yaml-cpp/yaml.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include "leveldb/slice.h"

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

    virtual upnsVec< upnsString > listCheckoutNames();
    virtual upnsVec< upnsSharedPointer<CheckoutObj> > listCheckouts();
    virtual upnsSharedPointer<CheckoutObj> getCheckoutCommit(const upnsString &name);
    virtual StatusCode storeCheckoutCommit(upnsSharedPointer<CheckoutObj> &obj, const upnsString &name);
    virtual StatusCode createCheckoutCommit(upnsSharedPointer<CheckoutObj> &obj, const upnsString &name);
    virtual StatusCode removeCheckoutCommit(const upnsString &name);

    virtual upnsVec< upnsSharedPointer<Branch> > listBranches();
    virtual upnsSharedPointer<Branch> getBranch(const upnsString &name);
    virtual StatusCode storeBranch(upnsSharedPointer<Branch> &obj, const upnsString &name);
    virtual StatusCode createBranch(upnsSharedPointer<Branch> &obj, const upnsString &name);
    virtual StatusCode removeBranch(const upnsString &name);

    virtual upnsSharedPointer<AbstractEntityDataStreamProvider> getStreamProvider(const ObjectId &entityId, bool canRead, bool canWrite);

    /**
     * @brief cleanUp Collects Grabage. Orphan Objects, not reachable by any branch are removed.
     * @return
     */
    virtual StatusCode cleanUp();

    virtual MessageType typeOfObject(const ObjectId &oidOrName);
    virtual bool exists(const ObjectId &oidOrName);
    // Please use getX and check for NULL!
//    virtual bool isTree(const ObjectId &oid);
//    virtual bool isEntity(const ObjectId &oid);
//    virtual bool isCommit(const CommitId &oid);
//    virtual bool isCheckout(const CommitId &oid);
//    virtual bool isBranch(const CommitId &oid);
private:
    leveldb::DB* m_db;

    StatusCode levelDbStatusToUpnsStatus(const leveldb::Status &levelDbStatus);
    QLockFile *m_lockFile;

    std::string keyOfTree(const ObjectId &oid) const;
    std::string keyOfEntity(const ObjectId &oid) const;
    std::string keyOfEntityData(const ObjectId &oid) const;
    std::string keyOfCommit(const ObjectId &oid) const;
    std::string keyOfCheckoutCommit(const upnsString &name) const;
    std::string keyOfBranch(const upnsString &name) const;

    StatusCode getObject(const std::string &key, std::string &value);
    StatusCode getObject(const leveldb::Slice &key, std::string &value);
    StatusCode getGenericEntryFromOid(const ObjectId &oidOrName, GenericEntry &value);
    StatusCode getGenericEntry(const std::string &key, GenericEntry &value);
    StatusCode getGenericEntry(const leveldb::Slice &key, GenericEntry &value);
    StatusCode storeObject(const std::string &key, const std::string &value);
    StatusCode createObject(const std::string &key, const std::string &value);
    StatusCode removeObject(const std::string &oid);

    template <typename T>
    upnsSharedPointer<T> getObject(const std::string &key);

    template <typename T>
    upnsSharedPointer<T> fromGeneric(const GenericEntry &from);

    template <typename T>
    StatusCode storeObject(const std::string &key, upnsSharedPointer<T> value);

    template <typename T>
    StatusCode createObject(const std::string &key, upnsSharedPointer<T> value);
};

}
#endif
