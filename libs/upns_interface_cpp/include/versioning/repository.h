#ifndef REPOSITORY_H
#define REPOSITORY_H

#include "upns_typedefs.h"
#include "services.pb.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include "entitydata.h"
#include "checkout.h"

namespace upns
{
class RepositoryPrivate;

/**
 * @brief The Repository class is the main interface to maps and data inside a repository.
 * Repository has functionality to work with data across their versions. If only a single/
 * the newest version of maps is needed for a user, the only methods needed from this interface
 * are "listCheckoutNames", "createCheckout" and "getCheckout". @sa Checkout provides all
 * functionality to read and write a specific version of objects.
 * Methods should never be called in a main or GUI Thread, as they may prevent the application
 * to respond to OS messages and thus freeze GUIs. Implementations of this interface will likely
 * be NOT thread-safe. About reentrancy:
 * 1. Multiple instances of the Repository class should not access the same underlying
 *    Repository-data at once.
 * 2. Multiple Repositories with different configurations are possible/must be possible with
 *    the implementation.
 * To decouple the application from different repository-implementations, methods should be used
 * as if they were NOT thread-safe and multiple Repository-instances for the same underlying data
 * should NOT be used.
 *
 * At the moment there are two implementations: 1) local, 2) network-stub/cached
 * Both implementations provide factories to instantiate Repository.
 */

class Repository
{
public:
    virtual ~Repository() {}
    /**
     * @brief getCheckouts retrieves a list of all checkouts in the system.
     * In contrast to git (with a single file tree), upns can checkout multiple versions at the same time.
     * This forces streams/entitydata to be physically stored locally (Is this a good idea? What about replaying operations?).
     * A commitId can only be checked out once. If multiple checkouts for a commit is wanted, the first one should
     * finish its work, so a new commit is generated (and the branch-tag is forwared). The the old commitId can be used.
     * @return all the checkouts
     */
    virtual upnsVec<upnsString> listCheckoutNames() = 0;


    virtual upnsSharedPointer<Tree> getTree(const ObjectId &oid) = 0;
    virtual upnsSharedPointer<Entity> getEntity(const ObjectId &oid) = 0;
    virtual upnsSharedPointer<Commit> getCommit(const ObjectId &oid) = 0;
    virtual upnsSharedPointer<CheckoutObj> getCheckoutObj(const upnsString &name) = 0;
    virtual upnsSharedPointer<Branch> getBranch(const upnsString &name) = 0;
    virtual MessageType typeOfObject(const ObjectId &oid) = 0;

    /**
     * @brief getEntitydataReadOnly reads an object, without checkout
     * @param oid
     * @return
     */
    virtual upnsSharedPointer<AbstractEntitydata> getEntitydataReadOnly(const ObjectId &oid) = 0;

    /**
     * @brief checkout creates a new checkout from a commit.
     * name not existing: create new commit
     * name already existing: error (returns null).
     * @param commitId
     * @param name
     * @return
     */
    virtual upnsSharedPointer<Checkout> createCheckout(const CommitId &commitIdOrBranchname, const upnsString &name) = 0;
    //upnsSharedPointer<Checkout> checkout(const upnsSharedPointer<Branch> &branch, const upnsString &name) = 0;
    /**
     * @brief checkout a commit. The Checkout-Object makes all data in the checked out version accessible.
     * Changes are not fully recorded at this level. Individual Stream-writes are recorded, without knowing the "OperationDescriptor".
     * This enables operator-modules to use the "Checkout"-Class for their implementations. Another level must version append this
     * versioning information to contain also "OperationDescriptor".
     * After a Checkout was commited, it points to the old uncommited state and should not be used afterwards. A new, blank checkout
     * must be done on the new commit (Note that this unintuitive behaviour should be hidden by one layer from the user).
     * If a commit was checked out, that also is a branch, the branch is forwarded on the first commit
     * (only if the parent commit still is the branch head).
     * @param commit
     * @return new empty checkout object, representing exactly the state of <commit>.
     */
    virtual upnsSharedPointer<Checkout> getCheckout(const upnsString &checkoutName) = 0;
    // when commiting we check if we are on a branch head. if so... update branch pointer
    //upnsSharedPointer<Checkout> checkout(const upnsSharedPointer<Branch> &commit) = 0;

    /**
     * @brief deleteCheckoutForced Deletes a checkout forever. This cannot be undone, like deleting changed files in git
     * @param checkout to delete
     * @return status
     */
    virtual StatusCode deleteCheckoutForced(const upnsString &checkoutName) = 0;

    /**
     * @brief commit Commits checked out data. The checkout must not be used after committing it. TODO: checkout should be updated to be based on new commit.
     * @param checkout
     * @param msg
     * @return commitId of new commit.
     */
    virtual CommitId commit(const upnsSharedPointer<Checkout> checkout, upnsString msg) = 0;

    /**
     * @brief getBranches List all Branches
     * @return all Branches, names with their current HEAD commitIds.
     */
    virtual upnsVec< upnsSharedPointer<Branch> > getBranches() = 0;

    /**
     * @brief push alls branches to <repo>
     * @param repo Other Repository with AbstractMapSerializer (maybe Network behind it?)
     * @return status
     */
    virtual StatusCode push(Repository &repo) = 0;

    /**
     * @brief pull TODO: same as <repo>.push(this) ???
     * @param repo
     * @return status
     */
    virtual StatusCode pull(Repository &repo) = 0;

    /**
     * @brief parseCommitRef Utility function to parse userinput like "origin/master~~^"
     * @param commitRef string
     * @return found commitId or InvalidCommitId
     */
    virtual CommitId parseCommitRef(const upnsString &commitRef) = 0;

    /**
     * @brief merge two commits. TODO: merge vs. rebase. Based on Changed data or "replay" operations.
     * @param mine
     * @param theirs
     * @param base
     * @return A checkout in conflict mode.
     */
    virtual upnsSharedPointer<Checkout> merge(const CommitId mine, const CommitId theirs, const CommitId base) = 0;

    /**
     * @brief ancestors retrieves all (or all until <level>) ancestors of an object. Note that a merged object has more parent.
     * If an object <oId1> has more than one parent <numParents> and ancestor( oId1, 1) is called, the retrieved list will have
     * <numParents> entries. This way parent siblings can be distinguishd from a hierarchy of parents.
     * @param commitId with the objectId. To put the objectId into context. An objectId might be referenced by multiple commits.
     * @param objectId
     * @param level
     * @return A List off commits with objectsIds, the complete (or up to <level>) history of an object
     */
    virtual upnsVec< upnsPair<CommitId, ObjectId> > ancestors(const CommitId &commitId, const ObjectId &objectId, const int level = 0) = 0;

    virtual bool canRead() = 0;
    virtual bool canWrite() = 0;
};

}
#endif
