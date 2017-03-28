#ifndef ABSTRACTSERIALIZER_H
#define ABSTRACTSERIALIZER_H

#include <upns/typedefs.h>
#include <upns/services.pb.h>
#include <upns/operators/serialization/abstractentitydataprovider.h>
#include <upns/entitydata.h>
#include <upns/errorcodes.h>

namespace upns
{
// Path with leading checkout name
typedef Path PathInternal;
/**
 * @brief The AbstractSerializer class capsulates data access. From outside maps and entities can be read/written.
 * However, the enduser should use a more convenient interface (versioning). This class is abstract and must be implemented by a
 * concrete class which has access to a storage (harddrive, files, network, database, other MapSerializer, ...).
 * Mapmanager is the only code that should know about this interface. It is located in the "include" folder for tests and stubs.
 */
class AbstractSerializer
{
public:
    virtual ~AbstractSerializer() {}
    virtual bool canRead() = 0;
    virtual bool canWrite() = 0;

    virtual std::shared_ptr<Tree> getTree(const ObjectId &oid) = 0;
    virtual std::shared_ptr<Tree> getTreeTransient(const PathInternal &transientId) = 0;
    // Note: storing and creating is only distinguished for transient oid (paths). When
    //       Hashes are used, the system does not know if a tree/entity with the same hash
    //       already exists or if it is a new tree/entity
    virtual std::pair<StatusCode, ObjectId> storeTree(std::shared_ptr<Tree> &obj) = 0;
    virtual std::pair<StatusCode, ObjectId> storeTreeTransient(std::shared_ptr<Tree> &obj, const PathInternal &transientId) = 0;
    //virtual StatusCode createTree(std::shared_ptr<Tree> &obj) = 0;
    virtual StatusCode removeTree(const ObjectId &oid) = 0;

    virtual std::shared_ptr<Entity> getEntity(const ObjectId oid) = 0;
    virtual std::shared_ptr<Entity> getEntityTransient(const PathInternal path) = 0;
    virtual std::pair<StatusCode, ObjectId> storeEntity(std::shared_ptr<Entity> &obj) = 0;
    virtual std::pair<StatusCode, ObjectId> storeEntityTransient(std::shared_ptr<Entity> &obj, const PathInternal &transientId) = 0;
    //virtual StatusCode createEntity(std::shared_ptr<Entity> &obj) = 0;
    virtual StatusCode removeEntity(const ObjectId &oid) = 0;

    virtual std::shared_ptr<Commit> getCommit(const ObjectId &oid) = 0;
    //virtual StatusCode storeCommit(std::shared_ptr<Commit> &obj) = 0;
    virtual std::pair<StatusCode, ObjectId> createCommit(std::shared_ptr<Commit> &obj) = 0;
    virtual StatusCode removeCommit(const ObjectId &oid) = 0;

    virtual std::vector< std::string > listCheckoutNames() = 0;
    virtual std::vector< std::shared_ptr<CheckoutObj> > listCheckouts() = 0;
    virtual std::shared_ptr<CheckoutObj> getCheckoutCommit(const std::string &name) = 0;
    virtual StatusCode storeCheckoutCommit(std::shared_ptr<CheckoutObj> &obj, const std::string &name) = 0;
    virtual StatusCode createCheckoutCommit(std::shared_ptr<CheckoutObj> &obj, const std::string &name) = 0;
    virtual StatusCode removeCheckoutCommit(const std::string &name) = 0;

    virtual std::vector< std::shared_ptr<Branch> > listBranches() = 0;
    virtual std::shared_ptr<Branch> getBranch(const std::string &name) = 0;
    virtual StatusCode storeBranch(std::shared_ptr<Branch> &obj, const std::string &name) = 0;
    virtual StatusCode createBranch(std::shared_ptr<Branch> &obj, const std::string &name) = 0;
    virtual StatusCode removeBranch(const std::string &name) = 0;

    virtual std::shared_ptr<AbstractEntitydataProvider> getStreamProvider(const ObjectId &entityId, bool canRead = true) = 0;
    virtual std::shared_ptr<AbstractEntitydataProvider> getStreamProviderTransient(const Path &path, bool canRead = true, bool canWrite = false) = 0;


    virtual MessageType typeOfObject(const ObjectId &oid) = 0;
    virtual MessageType typeOfObjectTransient(const PathInternal &path) = 0;
    virtual bool exists(const ObjectId &oidOrName) = 0;

    virtual std::pair<StatusCode, ObjectId> persistTransientEntitydata(const PathInternal &path) = 0;
//    virtual bool isTree(const ObjectId &oid) = 0;
//    virtual bool isEntity(const ObjectId &oid) = 0;
//    virtual bool isCommit(const CommitId &oid) = 0;
//    virtual bool isCheckout(const CommitId &oid) = 0;
//    virtual bool isBranch(const CommitId &oid) = 0;
    /**
     * @brief cleanUp Collects Grabage. Orphan Objects, not reachable by any branch are removed.
     * @return
     */
    virtual StatusCode cleanUp() = 0;
#ifdef UPNS_DEBUG
    virtual void debugDump() = 0;
#endif
};

}
#endif
