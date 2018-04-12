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
#include <mapit/time/time.h>
#include "workspace.h"

namespace mapit
{
class RepositoryPrivate;

/**
 * @brief The Repository class is the main interface to maps and data inside a repository.
 * Repository has functionality to work with data across their versions. If only a single/
 * the newest version of maps is needed for a user, the only methods needed from this interface
 * are "listWorkspaceNames", "createWorkspace" and "getWorkspace". @sa Workspace provides all
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
     * @brief getWorkspaces retrieves a list of all workspaces in the system.
     * In contrast to git (with a single file tree), mapit can workspace multiple versions at the same time.
     * This forces streams/entitydata to be physically stored locally (Is this a good idea? What about replaying operations?).
     * A commitId can only be checked out once. If multiple workspaces for a commit is wanted, the first one should
     * finish its work, so a new commit is generated (and the branch-tag is forwared). The the old commitId can be used.
     * @return all the workspaces
     */
    virtual std::vector<std::string> listWorkspaceNames() = 0;


    virtual std::shared_ptr<mapit::msgs::Tree> getTree(const ObjectId &oid) = 0;
    virtual std::shared_ptr<mapit::msgs::Entity> getEntity(const ObjectId &oid) = 0;
    virtual std::shared_ptr<mapit::msgs::Commit> getCommit(const CommitId &coID) = 0;
    virtual std::shared_ptr<mapit::msgs::WorkspaceObj> getWorkspaceObj(const std::string &name) = 0;
    virtual std::shared_ptr<mapit::msgs::Branch> getBranch(const std::string &name) = 0;
    virtual mapit::msgs::MessageType typeOfObject(const ObjectId &oid) = 0;

    /**
     * @brief getEntitydataReadOnly reads an object, without workspace
     * @param oid
     * @return
     */
    virtual std::shared_ptr<AbstractEntitydata> getEntitydataReadOnly(const ObjectId &oid) = 0;

    /**
     * @brief workspace creates a new workspace from a commit.
     * name not existing: create new commit
     * name already existing: error (returns null).
     * @param commitId
     * @param name
     * @return
     */
    virtual std::shared_ptr<Workspace> createWorkspace(const CommitId &commitIdOrBranchname, const std::string &name) = 0;
    //std::shared_ptr<Workspace> workspace(const std::shared_ptr<Branch> &branch, const std::string &name) = 0;
    /**
     * @brief workspace a commit. The Workspace-Object makes all data in the checked out version accessible.
     * Changes are not fully recorded at this level. Individual Stream-writes are recorded, without knowing the "OperationDescriptor".
     * This enables operator-modules to use the "Workspace"-Class for their implementations. Another level must version append this
     * versioning information to contain also "OperationDescriptor".
     * After a Workspace was commited, it points to the old uncommited state and should not be used afterwards. A new, blank workspace
     * must be done on the new commit (Note that this unintuitive behaviour should be hidden by one layer from the user).
     * If a commit was checked out, that also is a branch, the branch is forwarded on the first commit
     * (only if the parent commit still is the branch head).
     * @param commit
     * @return new empty workspace object, representing exactly the state of <commit>.
     */
    virtual std::shared_ptr<Workspace> getWorkspace(const std::string &workspaceName) = 0;
    // when commiting we check if we are on a branch head. if so... update branch pointer
    //std::shared_ptr<Workspace> workspace(const std::shared_ptr<Branch> &commit) = 0;

    /**
     * @brief deleteWorkspaceForced Deletes a workspace forever. This cannot be undone, like deleting changed files in git
     * @param workspace to delete
     * @return status
     */
    virtual StatusCode deleteWorkspaceForced(const std::string &workspaceName) = 0;

    /**
     * @brief commit Commits checked out data.
     * @param workspace
     * @param msg
     * @param author
     * @param email
     * @param stamp
     * @return commitId of new commit.
     */
    virtual CommitId commit(const std::shared_ptr<Workspace> workspace
                            , std::string msg
                            , std::string author
                            , std::string email
                            , mapit::time::Stamp stamp = mapit::time::Clock::now()) = 0;

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
     * @return A workspace in conflict mode.
     */
    virtual std::shared_ptr<Workspace> merge(const CommitId mine, const CommitId theirs, const CommitId base) = 0;

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
