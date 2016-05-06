#ifndef FS_SERIALIZER_H
#define FS_SERIALIZER_H

#include "upns_globals.h"
#include "serialization/abstractmapserializer.h"
#include "yaml-cpp/yaml.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include <boost/filesystem.hpp>

class QLockFile;
namespace leveldb {
  class DB;
  class Status;
}
namespace fs = boost::filesystem;

namespace upns
{

  /**
 * @brief The LevelDBSerializer class stores all data using leveldb.
 *
 */

  class FSSerializer : public AbstractMapSerializer
  {
  private:
    static const std::string _PREFIX_MAPIT_;
    static const std::string _PREFIX_TREE_;
    static const std::string _PREFIX_ENTITY_;
    static const std::string _PREFIX_CHECKOUTS_;
    static const std::string _PREFIX_BRANCHES_;
    static const std::string _PREFIX_COMMIT_;

    static const std::string _FILE_CHECKOUT_;
  private:
    fs::path repo_;
  private:
    void fs_check_create(fs::path path);
    void fs_write(fs::path path, std::string value);
    void fs_read(fs::path path, upnsSharedPointer<GenericEntry> entry);
  public:

    FSSerializer(const YAML::Node &config);
    virtual ~FSSerializer();

    virtual bool canRead();
    virtual bool canWrite();

    virtual upnsSharedPointer<Tree> getTree(const ObjectId &oid);
    virtual upnsSharedPointer<Tree> getTreeTransient(const ObjectId &transientId);
    virtual upnsPair<StatusCode, ObjectId> storeTree(upnsSharedPointer<Tree> &obj);
    virtual upnsPair<StatusCode, ObjectId> storeTreeTransient(upnsSharedPointer<Tree> &obj, const ObjectId &transientId);
    //virtual upnsPair<StatusCode, ObjectId> createTreeTransient(upnsSharedPointer<Tree> &obj, const Path &path);
    virtual StatusCode removeTree(const ObjectId &oid);

    virtual upnsSharedPointer<Entity> getEntity(const ObjectId oid);
    virtual upnsSharedPointer<Entity> getEntityTransient(const ObjectId oid);
    virtual upnsPair<StatusCode, ObjectId> storeEntity(upnsSharedPointer<Entity> &obj);
    virtual upnsPair<StatusCode, ObjectId> storeEntityTransient(upnsSharedPointer<Entity> &obj, const ObjectId &transientId);
    //virtual StatusCode createEntityTransient(upnsSharedPointer<Entity> &obj);
    virtual StatusCode removeEntity(const ObjectId &oid);

    virtual upnsSharedPointer<Commit> getCommit(const ObjectId &oid);
    //virtual upnsPair<StatusCode, ObjectId> storeCommit(upnsSharedPointer<Commit> &obj);
    virtual upnsPair<StatusCode, ObjectId> createCommit(upnsSharedPointer<Commit> &obj);
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

    virtual upnsPair<StatusCode, ObjectId> persistTransientEntityData(const ObjectId &entityId);
#ifdef UPNS_DEBUG
    virtual void debugDump();
#endif
  };

}
#endif
