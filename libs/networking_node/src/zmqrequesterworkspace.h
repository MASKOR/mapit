/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef ZMQREQUESTERWORKSPACE_H
#define ZMQREQUESTERWORKSPACE_H

#include <string>
#include <mapit/versioning/repository.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include "zmqprotobufnode.h"
#include <mutex>

namespace mapit {

///
/// \brief The ZmqRequesterWorkspace class
/// Implements the basic Workspace Interface and will send requests over network
/// Compute local:
/// - true: makes it possible to read from filesystem locally, and write to remote repository
/// - false: makes it possible to read from remote filesystem to remote repo
///

class ZmqRequesterWorkspace : public mapit::Workspace, public mapit::operators::WorkspaceWritable
{
public:
    ZmqRequesterWorkspace(std::string name, ZmqProtobufNode *node, mapit::Workspace *cache, bool operationsLocal, std::mutex *requestMutex);

    // WorkspaceCommon interface
public:
    virtual const std::string& getName();
    bool isInConflictMode();
    std::vector<std::shared_ptr<Conflict> > getPendingConflicts();
    void setConflictSolved(const Path &path, const ObjectId &oid);
    virtual MessageType typeOfObject(const Path &oidOrName);
    std::shared_ptr<Commit> getRollingcommit();
    std::shared_ptr<Tree> getRoot();
    std::shared_ptr<Tree> getTreeConflict(const ObjectId &objectId);
    std::shared_ptr<Entity> getEntityConflict(const ObjectId &objectId);
    std::shared_ptr<Tree> getTree(const Path &path);
    std::shared_ptr<Entity> getEntity(const Path &path);
    std::shared_ptr<Branch> getParentBranch();
    std::vector<CommitId> getParentCommitIds();
    std::shared_ptr<AbstractEntitydata> getEntitydataReadOnly(const Path &entityId);
    std::shared_ptr<AbstractEntitydata> getEntitydataReadOnlyConflict(const ObjectId &entityId);
    StatusCode depthFirstSearch(  std::function<bool (std::shared_ptr<Tree>, const ObjectReference &, const Path &)> beforeTree
                                , std::function<bool (std::shared_ptr<Tree>, const ObjectReference &, const Path &)> afterTree
                                , std::function<bool (std::shared_ptr<Entity>, const ObjectReference &, const Path &)> beforeEntity
                                , std::function<bool (std::shared_ptr<Entity>, const ObjectReference &, const Path &)> afterEntity);
    StatusCode depthFirstSearch(  const Path& path
                                , std::function<bool(std::shared_ptr<mapit::msgs::Tree>, const mapit::msgs::ObjectReference&, const Path&)> beforeTree
                                , std::function<bool(std::shared_ptr<mapit::msgs::Tree>, const mapit::msgs::ObjectReference&, const Path&)> afterTree
                                , std::function<bool(std::shared_ptr<mapit::msgs::Entity>, const mapit::msgs::ObjectReference&, const Path&)> beforeEntity
                                , std::function<bool(std::shared_ptr<mapit::msgs::Entity>, const mapit::msgs::ObjectReference&, const Path&)> afterEntity);
    // Workspace interface
public:
    OperationResult doOperation(const OperationDescription &desc);
    mapit::StatusCode storeOperationDesc_(const OperationDescription &desc, bool restorable);
    OperationResult doUntraceableOperation(const OperationDescription &desc, std::function<mapit::StatusCode(mapit::OperationEnvironment*)> operate);

    // WorkspaceWritable interface
public:
    StatusCode storeTree(const Path &path, std::shared_ptr<Tree> tree);
    StatusCode storeEntity(const Path &path, std::shared_ptr<Entity> entity);
    virtual StatusCode deleteTree(const Path &path);
    virtual StatusCode deleteEntity(const Path &path);
    std::shared_ptr<AbstractEntitydata> getEntitydataForReadWrite(const Path &entity);

private:
    std::string m_workspaceName;
    ZmqProtobufNode *m_node;
    mapit::Workspace *m_cache;
    bool m_operationsLocal;

    //void syncHierarchy();


    std::mutex *m_requestMutex; //TODO: this is only needed for req/rep pattern
};

}

#endif // ZMQREQUESTERWORKSPACE_H
