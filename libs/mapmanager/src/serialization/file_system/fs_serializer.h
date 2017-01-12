#ifndef FS_SERIALIZER_H
#define FS_SERIALIZER_H

#include "upns_typedefs.h"
#include "upns_logging.h"
#include "serialization/abstractmapserializer.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

class QLockFile;

namespace fs = boost::filesystem;

namespace upns
{

  /**
 * @brief The FSSerializer class stores all data using a file system.
 *
 */

  class FSSerializer : public AbstractMapSerializer
  {
  private:
    static const fs::path _PREFIX_MAPIT_;
    static const fs::path _PREFIX_TREE_;
    static const fs::path _PREFIX_ENTITY_;
    static const fs::path _PREFIX_ENTITY_DATA_;
    static const fs::path _PREFIX_CHECKOUTS_;
    static const fs::path _PREFIX_BRANCHES_;
    static const fs::path _PREFIX_COMMIT_;

//    static const fs::path _FILE_CHECKOUT_;
    static const fs::path _PREFIX_CHECKOUT_;
    static const fs::path _CHECKOUT_GENERIC_ENTRY_;
    static const fs::path _CHECKOUT_ENTITY_DATA_;
    static const fs::path _CHECKOUT_ROOT_FOLDER_;
  private:
    fs::path repo_;
  private:
    fs::path objectid_to_checkout_fs_path(ObjectId oid);
    void fs_check_create(fs::path path);
    void fs_write(fs::path path, upns::upnsSharedPointer<GenericEntry> ge, MessageType msgType, bool overwrite = false);
    void fs_read(fs::path path, upnsSharedPointer<GenericEntry> entry);
  public:

    FSSerializer(std::__cxx11::string directory);
    virtual ~FSSerializer();

    virtual bool canRead();
    virtual bool canWrite();

    virtual upnsSharedPointer<Tree> getTree(const ObjectId &oid);
    virtual upnsSharedPointer<Tree> getTreeTransient(const PathInternal &transientId);
    virtual upnsPair<StatusCode, ObjectId> storeTree(upnsSharedPointer<Tree> &obj);
    virtual upnsPair<StatusCode, ObjectId> storeTreeTransient(upnsSharedPointer<Tree> &obj, const PathInternal &transientId);
    //virtual upnsPair<StatusCode, ObjectId> createTreeTransient(upnsSharedPointer<Tree> &obj, const Path &path);
    virtual StatusCode removeTree(const ObjectId &oid);

    virtual upnsSharedPointer<Entity> getEntity(const ObjectId oid);
    virtual upnsSharedPointer<Entity> getEntityTransient(const PathInternal oid);
    virtual upnsPair<StatusCode, ObjectId> storeEntity(upnsSharedPointer<Entity> &obj);
    virtual upnsPair<StatusCode, ObjectId> storeEntityTransient(upnsSharedPointer<Entity> &obj, const PathInternal &transientId);
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

    virtual upnsSharedPointer<AbstractEntitydataStreamProvider> getStreamProvider(const ObjectId &entityId, bool canRead);
    virtual upnsSharedPointer<AbstractEntitydataStreamProvider> getStreamProviderTransient(const Path &oid, bool canRead, bool canWrite);

    /**
     * @brief cleanUp Collects Grabage. Orphan Objects, not reachable by any branch are removed.
     * @return
     */
    virtual StatusCode cleanUp();

    virtual MessageType typeOfObject(const ObjectId &oid);
    virtual MessageType typeOfObjectTransient(const PathInternal &pathIntenal);
    virtual bool exists(const ObjectId &oidOrName);

    virtual upnsPair<StatusCode, ObjectId> persistTransientEntitydata(const PathInternal &pathInternal);
#ifdef UPNS_DEBUG
    virtual void debugDump();
#endif
  };

}
#endif
