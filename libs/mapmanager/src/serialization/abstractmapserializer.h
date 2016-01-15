#ifndef ABSTRACTMAPSERIALIZER_H
#define ABSTRACTMAPSERIALIZER_H

#include "upns_globals.h"
#include "services.pb.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include "entitydata.h"
#include "error.h"

namespace upns
{

/**
 * @brief The AbstractMapSerializer class capsulates data access. From outside maps and entities can be read/written.
 * However, the enduser use a more convenient interface (versioning). This class is abstract and must be implemented by a
 * concrete class which has access to a storage (harddrive, files, netrwork, database, other MapSerializer, ...).
 */
class AbstractMapSerializer
{
public:
    virtual bool canRead() = 0;
    virtual bool canWrite() = 0;

    virtual upnsSharedPointer<Tree> getTree(const ObjectId &oid) = 0;
    virtual StatusCode storeTree(upnsSharedPointer<Tree> &obj) = 0;
    virtual StatusCode createTree(upnsSharedPointer<Tree> &obj) = 0;
    virtual StatusCode removeTree(const ObjectId &oid) = 0;

    virtual upnsSharedPointer<Entity> getEntity(const ObjectId oid) = 0;
    virtual StatusCode storeEntity(upnsSharedPointer<Entity> &obj) = 0;
    virtual StatusCode createEntity(upnsSharedPointer<Entity> &obj) = 0;
    virtual StatusCode removeEntity(const ObjectId &oid) = 0;

    virtual upnsSharedPointer<Commit> getCommit(const ObjectId &oid) = 0;
    virtual StatusCode storeCommit(upnsSharedPointer<Commit> &obj) = 0;
    virtual StatusCode createCommit(upnsSharedPointer<Commit> &obj) = 0;
    virtual StatusCode removeCommit(const ObjectId &oid) = 0;

    virtual upnsVec< upnsString > listCheckoutNames() = 0;
    virtual upnsVec< upnsSharedPointer<CheckoutObj> > listCheckouts() = 0;
    virtual upnsSharedPointer<CheckoutObj> getCheckoutCommit(const upnsString &name) = 0;
    virtual StatusCode storeCheckoutCommit(upnsSharedPointer<CheckoutObj> &obj, const upnsString &name) = 0;
    virtual StatusCode createCheckoutCommit(upnsSharedPointer<CheckoutObj> &obj, const upnsString &name) = 0;
    virtual StatusCode removeCheckoutCommit(const upnsString &name) = 0;

    virtual upnsVec< upnsSharedPointer<Branch> > listBranches() = 0;
    virtual upnsSharedPointer<Branch> getBranch(const upnsString &name) = 0;
    virtual StatusCode storeBranch(upnsSharedPointer<Branch> &obj, const upnsString &name) = 0;
    virtual StatusCode createBranch(upnsSharedPointer<Branch> &obj, const upnsString &name) = 0;
    virtual StatusCode removeBranch(const upnsString &name) = 0;

    virtual upnsSharedPointer<AbstractEntityDataStreamProvider> getStreamProvider(const ObjectId &entityId, bool canRead = true, bool canWrite = false) = 0;

    virtual MessageType typeOfObject(const ObjectId &oidOrName) = 0;
    virtual bool exists(const ObjectId &oidOrName) = 0;

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
};

}
#endif
