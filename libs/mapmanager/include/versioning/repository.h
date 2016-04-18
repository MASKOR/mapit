#ifndef REPOSITORY_H
#define REPOSITORY_H

#include "upns_globals.h"
#include "services.pb.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include "entitydata.h"
#include "checkout.h"
#include "yaml-cpp/yaml.h"

namespace upns
{
class RepositoryPrivate;

class Repository
{
public:
    Repository(const YAML::Node &config);
    ~Repository();

    /**
     * @brief getCheckouts retrieves a list of all checkouts in the system.
     * In contrast to git (with a single file tree), upns can checkout multiple versions at the same time.
     * This forces streams/entitydata to be physically stored locally (Is this a good idea? What about replaying operations?).
     * A commitId can only be checked out once. If multiple checkouts for a commit is wanted, the first one should
     * finish its work, so a new commit is generated (and the branch-tag is forwared). The the old commitId can be used.
     * @return all the checkouts
     */
    upnsVec<upnsString> listCheckoutNames();


    upnsSharedPointer<Tree> getTree(const ObjectId &oid);
    upnsSharedPointer<Entity> getEntity(const ObjectId &oid);
    upnsSharedPointer<Commit> getCommit(const ObjectId &oid);
    upnsSharedPointer<CheckoutObj> getCheckoutObj(const upnsString &name);
    upnsSharedPointer<Branch> getBranch(const upnsString &name);
    MessageType typeOfObject(const ObjectId &oid);

    /**
     * @brief getEntityDataReadOnly reads an object, without checkout
     * @param oid
     * @return
     */
    upnsSharedPointer<AbstractEntityData> getEntityDataReadOnly(const ObjectId &oid);

    /**
     * @brief checkout creates a new checkout from a commit.
     * name not existing: create new commit
     * name already existing: error (returns null).
     * @param commitId
     * @param name
     * @return
     */
    upnsSharedPointer<Checkout> createCheckout(const CommitId &commitIdOrBranchname, const upnsString &name);
    //upnsSharedPointer<Checkout> checkout(const upnsSharedPointer<Branch> &branch, const upnsString &name);
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
    upnsSharedPointer<Checkout> getCheckout(const upnsString &checkoutName);
    // when commiting we check if we are on a branch head. if so... update branch pointer
    //upnsSharedPointer<Checkout> checkout(const upnsSharedPointer<Branch> &commit);

    /**
     * @brief deleteCheckoutForced Deletes a checkout forever. This cannot be undone, like deleting changed files in git
     * @param checkout to delete
     * @return status
     */
    StatusCode deleteCheckoutForced(const upnsString &checkoutName);

    /**
     * @brief commit Commits checked out data. The checkout must not be used after committing it. TODO: checkout should be updated to be based on new commit.
     * @param checkout
     * @param msg
     * @return commitId of new commit.
     */
    CommitId commit(const upnsSharedPointer<Checkout> checkout, const upnsString msg);

    /**
     * @brief getBranches List all Branches
     * @return all Branches, names with their current HEAD commitIds.
     */
    upnsVec< upnsSharedPointer<Branch> > getBranches();

    /**
     * @brief push alls branches to <repo>
     * @param repo Other Repository with AbstractMapSerializer (maybe Network behind it?)
     * @return status
     */
    StatusCode push(Repository &repo);

    /**
     * @brief pull TODO: same as <repo>.push(this) ???
     * @param repo
     * @return status
     */
    StatusCode pull(Repository &repo);

    /**
     * @brief parseCommitRef Utility function to parse userinput like "origin/master~~^"
     * @param commitRef string
     * @return found commitId or InvalidCommitId
     */
    CommitId parseCommitRef(const upnsString &commitRef);

    /**
     * @brief merge two commits. TODO: merge vs. rebase. Based on Changed data or "replay" operations.
     * @param mine
     * @param theirs
     * @param base
     * @return A checkout in conflict mode.
     */
    upnsSharedPointer<Checkout> merge(const CommitId mine, const CommitId theirs, const CommitId base);

    /**
     * @brief ancestors retrieves all (or all until <level>) ancestors of an object. Note that a merged object has more parent.
     * If an object <oId1> has more than one parent <numParents> and ancestor( oId1, 1) is called, the retrieved list will have
     * <numParents> entries. This way parent siblings can be distinguishd from a hierarchy of parents.
     * @param commitId with the objectId. To put the objectId into context. An objectId might be referenced by multiple commits.
     * @param objectId
     * @param level
     * @return A List off commits with objectsIds, the complete (or up to <level>) history of an object
     */
    upnsVec< upnsPair<CommitId, ObjectId> > ancestors(const CommitId &commitId, const ObjectId &objectId, const int level = 0);

    /**
     * @brief initialized the repository.
     * This must be called exactly once for new repositories.
     * It creates a master branch with an empty root tree.
     * @return
     */
    StatusCode init();

    bool canRead();
    bool canWrite();
private:
    RepositoryPrivate* m_p;
};

}
#endif
