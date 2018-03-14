/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAPIT_REPOSITORY_H
#define MAPIT_REPOSITORY_H

#include <mapit/typedefs.h>
#include <mapit/msgs/services.pb.h>
#include <mapit/operators/serialization/abstractentitydataprovider.h>
#include <mapit/entitydata.h>
#include "checkout.h"

namespace mapit
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
     * In contrast to git (with a single file tree), mapit can checkout multiple versions at the same time.
     * This forces streams/entitydata to be physically stored locally (Is this a good idea? What about replaying operations?).
     * A commitId can only be checked out once. If multiple checkouts for a commit is wanted, the first one should
     * finish its work, so a new commit is generated (and the branch-tag is forwared). The the old commitId can be used.
     * @return all the checkouts
     */
    virtual std::vector<std::string> listCheckoutNames() = 0;


    virtual std::shared_ptr<mapit::msgs::Tree> getTree(const ObjectId &oid) = 0;
    virtual std::shared_ptr<mapit::msgs::Entity> getEntity(const ObjectId &oid) = 0;
    virtual std::shared_ptr<mapit::msgs::Commit> getCommit(const ObjectId &oid) = 0;
    virtual std::shared_ptr<mapit::msgs::CheckoutObj> getCheckoutObj(const std::string &name) = 0;
    virtual std::shared_ptr<mapit::msgs::Branch> getBranch(const std::string &name) = 0;
    virtual mapit::msgs::MessageType typeOfObject(const ObjectId &oid) = 0;

    /**
     * @brief getEntitydataReadOnly reads an object, without checkout
     * @param oid
     * @return
     */
    virtual std::shared_ptr<AbstractEntitydata> getEntitydataReadOnly(const ObjectId &oid) = 0;

    /**
     * @brief checkout creates a new checkout from a commit.
     * name not existing: create new commit
     * name already existing: error (returns null).
     * @param commitId
     * @param name
     * @return
     */
    virtual std::shared_ptr<Checkout> createCheckout(const CommitId &commitIdOrBranchname, const std::string &name) = 0;
    //std::shared_ptr<Checkout> checkout(const std::shared_ptr<Branch> &branch, const std::string &name) = 0;
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
    virtual std::shared_ptr<Checkout> getCheckout(const std::string &checkoutName) = 0;
    // when commiting we check if we are on a branch head. if so... update branch pointer
    //std::shared_ptr<Checkout> checkout(const std::shared_ptr<Branch> &commit) = 0;

    /**
     * @brief deleteCheckoutForced Deletes a checkout forever. This cannot be undone, like deleting changed files in git
     * @param checkout to delete
     * @return status
     */
    virtual StatusCode deleteCheckoutForced(const std::string &checkoutName) = 0;

    /**
     * @brief commit Commits checked out data.
     * @param checkout
     * @param msg
     * @param author
     * @param email
     * @return commitId of new commit.
     */
    virtual CommitId commit(const std::shared_ptr<Checkout> checkout, std::string msg, std::string author, std::string email) = 0;

    /**
     * @brief getBranches List all Branches
     * @return all Branches, names with their current HEAD commitIds.
     */
    virtual std::vector< std::shared_ptr<mapit::msgs::Branch> > getBranches() = 0;

    /**
     * @brief push alls branches to <repo>
     * @param repo Other Repository with AbstractSerializer (maybe Network behind it?)
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
    virtual CommitId parseCommitRef(const std::string &commitRef) = 0;

    /**
     * @brief merge two commits. TODO: merge vs. rebase. Based on Changed data or "replay" operations.
     * @param mine
     * @param theirs
     * @param base
     * @return A checkout in conflict mode.
     */
    virtual std::shared_ptr<Checkout> merge(const CommitId mine, const CommitId theirs, const CommitId base) = 0;

    /**
     * @brief ancestors retrieves all (or all until <level>) ancestors of an object. Note that a merged object has more parent.
     * If an object <oId1> has more than one parent <numParents> and ancestor( oId1, 1) is called, the retrieved list will have
     * <numParents> entries. This way parent siblings can be distinguishd from a hierarchy of parents.
     * @param commitId with the objectId. To put the objectId into context. An objectId might be referenced by multiple commits.
     * @param objectId
     * @param level
     * @return A List off commits with objectsIds, the complete (or up to <level>) history of an object
     */
    virtual std::vector< std::pair<CommitId, ObjectId> > ancestors(const CommitId &commitId, const ObjectId &objectId, const int level = 0) = 0;

    virtual bool canRead() = 0;
    virtual bool canWrite() = 0;
};

}
#endif
