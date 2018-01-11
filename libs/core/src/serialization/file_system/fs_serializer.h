#ifndef FS_SERIALIZER_H
#define FS_SERIALIZER_H

#include <upns/typedefs.h>
#include <upns/logging.h>
#include "serialization/abstractserializer.h"
#include <upns/operators/serialization/abstractentitydataprovider.h>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

class QLockFile;

namespace fs = boost::filesystem;

using namespace mapit::msgs;

namespace upns
{


  /**
 * @brief The FSSerializer class stores all data using a file system.
 *
 */

  class FSSerializer : public AbstractSerializer
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
    void fs_write(fs::path path, std::shared_ptr<GenericEntry> ge, MessageType msgType, bool overwrite = false);
    void fs_read(fs::path path, std::shared_ptr<GenericEntry> entry);
    void fs_delete(fs::path path);
  public:

    FSSerializer(std::string directory);
    virtual ~FSSerializer();

    virtual bool canRead();
    virtual bool canWrite();

    virtual std::shared_ptr<Tree> getTree(const ObjectId &oid);
    virtual std::shared_ptr<Tree> getTreeTransient(const PathInternal &transientId);
    virtual std::pair<StatusCode, ObjectId> storeTree(std::shared_ptr<Tree> &obj);
    virtual std::pair<StatusCode, ObjectId> storeTreeTransient(std::shared_ptr<Tree> &obj, const PathInternal &transientId);
    //virtual std::pair<StatusCode, ObjectId> createTreeTransient(std::shared_ptr<Tree> &obj, const Path &path);
    virtual StatusCode removeTreeTransient(const PathInternal &transientId);

    virtual std::shared_ptr<mapit::msgs::Entity> getEntity(const ObjectId oid);
    virtual std::shared_ptr<mapit::msgs::Entity> getEntityTransient(const PathInternal oid);
    virtual std::pair<StatusCode, ObjectId> storeEntity(std::shared_ptr<mapit::msgs::Entity> &obj);
    virtual std::pair<StatusCode, ObjectId> storeEntityTransient(std::shared_ptr<mapit::msgs::Entity> &obj, const PathInternal &transientId);
    //virtual StatusCode createEntityTransient(std::shared_ptr<Entity> &obj);
    virtual StatusCode removeEntityTransient(const PathInternal &transientId);

    virtual std::shared_ptr<Commit> getCommit(const ObjectId &oid);
    //virtual std::pair<StatusCode, ObjectId> storeCommit(std::shared_ptr<Commit> &obj);
    virtual std::pair<StatusCode, ObjectId> createCommit(std::shared_ptr<Commit> &obj);
    virtual StatusCode removeCommit(const ObjectId &oid);

    virtual std::vector< std::string > listCheckoutNames();
    virtual std::vector< std::shared_ptr<CheckoutObj> > listCheckouts();
    virtual std::shared_ptr<CheckoutObj> getCheckoutCommit(const std::string &name);
    virtual StatusCode storeCheckoutCommit(std::shared_ptr<CheckoutObj> &obj, const std::string &name);
    virtual StatusCode createCheckoutCommit(std::shared_ptr<CheckoutObj> &obj, const std::string &name);
    virtual StatusCode removeCheckoutCommit(const std::string &name);

    virtual std::vector< std::shared_ptr<Branch> > listBranches();
    virtual std::shared_ptr<Branch> getBranch(const std::string &name);
    virtual StatusCode storeBranch(std::shared_ptr<Branch> &obj, const std::string &name);
    virtual StatusCode createBranch(std::shared_ptr<Branch> &obj, const std::string &name);
    virtual StatusCode removeBranch(const std::string &name);

    virtual std::shared_ptr<AbstractEntitydataProvider> getStreamProvider(const ObjectId &entityId, bool canRead);
    virtual std::shared_ptr<AbstractEntitydataProvider> getStreamProviderTransient(const Path &oid, bool canRead, bool canWrite);

    /**
     * @brief cleanUp Collects Grabage. Orphan Objects, not reachable by any branch are removed.
     * @return
     */
    virtual StatusCode cleanUp();

    virtual MessageType typeOfObject(const ObjectId &oid);
    virtual MessageType typeOfObjectTransient(const PathInternal &pathIntenal);
    virtual bool exists(const ObjectId &oidOrName);

    virtual std::pair<StatusCode, ObjectId> persistTransientEntitydata(const PathInternal &pathInternal);
#ifdef UPNS_DEBUG
    virtual void debugDump();
#endif
  };

}
#endif
