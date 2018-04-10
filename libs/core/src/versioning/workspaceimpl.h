/*******************************************************************************
 *
 * Copyright 2016-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2016-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef WORKSPACEIMPL_H
#define WORKSPACEIMPL_H

#include <mapit/typedefs.h>
#include <mapit/logging.h>
#include <mapit/msgs/services.pb.h>
#include <mapit/operators/serialization/abstractentitydataprovider.h>
#include "serialization/abstractserializer.h"
#include <mapit/entitydata.h>
#include <mapit/versioning/workspace.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include "util.h"
#include <functional>

using namespace mapit::msgs;

namespace mapit
{

// PathInternal contains the workspace name at the beginning
// The corresponing types should be used. However there is no "typechecking" here.
typedef Path PathInternal;
/**
 * @brief The WorkspaceImpl class
 * Why?
 *
 * Workspace vs. operators::WorkspaceWritable
 * The Class "Workspace" is meant to be used by user/application. The Class "WorkspaceWritable" is meant to be used by operators.
 * WorkspaceWritable can edit objects and can not execute other operators (thus, recursive execution of operators is forbidden, it could break metadata).
 * Workspace can not edit objects, as the application can only do changes by executing operations!
 *
 * WorkspaceCommon:
 * Both classes have similarities (e.g. all the read operations). These similarities are not repeated in both interfaces, but in "WorkspaceCommon".
 *
 * WorkspaceImpl:
 * When a Workspace is generated and given to an operator, the objects should not be copied. Thus, both of the classes ("Workspace" and "WorkspaceWritable")
 * need to be one object with both implementations. The implementation is in "WorkspaceImpl". Instances of "WorkspaceImpl" can be seen as "Workspace" for
 * the user/application and as "WorkspaceWritable" from an operator.
 *
 */
class WorkspaceImpl : public Workspace, public operators::WorkspaceWritable /*, WorkspaceCommon*/
{
public:
    /**
     * @brief Workspace represents an editable state of a group of entities.
     * @param serializer
     * @param commitOrCheckoutId
     */
    WorkspaceImpl(std::shared_ptr<AbstractSerializer> serializer, std::shared_ptr<WorkspaceObj> workspaceCommit, std::string name, const std::string branchname = "");

    virtual bool isInConflictMode();
    virtual std::vector< std::shared_ptr<Conflict> > getPendingConflicts();
    virtual std::shared_ptr<Commit> getRollingcommit();
    virtual std::shared_ptr<Tree> getRoot();
    virtual std::shared_ptr<Tree> getTree(const Path &path);
    virtual std::shared_ptr<Entity> getEntity(const Path &path);
    virtual MessageType typeOfObject(const Path &path);
    virtual std::shared_ptr<Tree> getTreeConflict(const ObjectId &objectId);
    virtual std::shared_ptr<Entity> getEntityConflict(const ObjectId &objectId);
    virtual OperationResult doOperation(const OperationDescription &desc);
    virtual OperationResult doUntraceableOperation(const OperationDescription &desc, std::function<mapit::StatusCode(OperationEnvironment *)> operate);

    virtual std::shared_ptr<AbstractEntitydata> getEntitydataReadOnly(const Path &path);
    virtual std::shared_ptr<AbstractEntitydata> getEntitydataReadOnlyConflict(const ObjectId &entityId);
    virtual std::shared_ptr<AbstractEntitydata> getEntitydataForReadWrite(const Path &path);

    virtual StatusCode storeTree(const Path &path, std::shared_ptr<Tree> tree);
    virtual StatusCode storeEntity(const Path &path, std::shared_ptr<Entity> entity);

    virtual StatusCode deleteTree(const Path &path);
    virtual StatusCode deleteEntity(const Path &path);

    virtual void setConflictSolved(const Path &path, const ObjectId &oid);

    virtual std::shared_ptr<Branch> getParentBranch();
    virtual std::vector<CommitId> getParentCommitIds();

    template <typename T>
    inline std::string generateTransientOid(const std::shared_ptr<T> &obj);

    StatusCode depthFirstSearch(  std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree
                                , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree
                                , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity
                                , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity);
    StatusCode depthFirstSearch(  const Path& path
                                , std::function<bool(std::shared_ptr<mapit::msgs::Tree>, const mapit::msgs::ObjectReference&, const Path&)> beforeTree
                                , std::function<bool(std::shared_ptr<mapit::msgs::Tree>, const mapit::msgs::ObjectReference&, const Path&)> afterTree
                                , std::function<bool(std::shared_ptr<mapit::msgs::Entity>, const mapit::msgs::ObjectReference&, const Path&)> beforeEntity
                                , std::function<bool(std::shared_ptr<mapit::msgs::Entity>, const mapit::msgs::ObjectReference&, const Path&)> afterEntity);
    std::shared_ptr<WorkspaceObj> getWorkspaceObj();
    const std::string& getName() const;
    const std::string& getBranchName() const;
private:

    std::shared_ptr<Tree> getTree(const ObjectReference &ref);
    std::shared_ptr<Entity> getEntity(const ObjectReference &ref);
    MessageType typeOfObject(const ObjectReference &ref);
    /**
     * @brief objectReferenceOfChild Used to get reference of children
     * @param tree parent
     * @param name of the child (path segment, without slashes)
     * @return Oid or empty oid
     */
    ObjectReference objectReferenceOfChild(std::shared_ptr<Tree> tree, const std::string &name);

    /**
     * @brief oidForPath Used to convert path to oids.
     * @param path beginning with root dir of workspace. Workspacenames must not be part of path. Can have leading or trailing slashes.
     * @return Oid or empty oid
     */
    ObjectReference objectReferenceForPath(const Path &path);

    // helper, used to ensure slashes/no slashes at beginning and end
    Path preparePath(const Path &path);
    Path preparePathFilename(const Path &path);

    /**
     * @brief createPath If workspace wants to write to a path, it must be created. The leaf can be a tree or entity. This is not a trivial function, but should be easy to use from the outside.
     * @param path to create in the workspace.
     * @param createLeaf null, if the leaf exists as non-exclusive tree/entity. Can also be a new entity. Can also be a tree (e.g. copy/move operation).
     */
    template <typename T>
    StatusCode createPath(const Path &path, std::shared_ptr<T> createLeaf = std::shared_ptr<T>(nullptr), bool deleteLeaf = false);

    /**
     * @brief Stores all kind of objects at a path.
     */
    template <typename T>
    std::pair<StatusCode, PathInternal> storeObject(std::shared_ptr<T> leafObject, const PathInternal &pathInternal);

    /**
     * @brief Delete all kind of objects at a path.
     */
    template <typename T>
    StatusCode deleteObject(const Path &path);

    bool forEachPathSegment(const Path &path
                            , std::function<bool(std::string, size_t, bool)> before
                            , std::function<bool(std::string, size_t, bool)> after
                            , const int start = 0);

//    /**
//     * @brief Depth first search for Commit, Tree and Entity.
//     * Does not work for branches. Does not visit Entitydata (must be done manually).
//     * If "before" returns false, "after" will not be executed.
//     */
//    StatusCode depthFirstSearch(std::shared_ptr<Commit> obj, const ObjectId& oid, const Path &path,
//                                std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> afterCommit,
//                                std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> afterTree,
//                                std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> afterEntity);

//    StatusCode depthFirstSearch(std::shared_ptr<Tree> obj, const ObjectId& oid, const Path &path,
//                                std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> afterCommit,
//                                std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> afterTree,
//                                std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> afterEntity);

//    StatusCode depthFirstSearch(std::shared_ptr<Entity> obj, const ObjectId& oid, const Path &path,
//                                std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> afterCommit,
//                                std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> afterTree,
//                                std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> afterEntity);

    std::shared_ptr<AbstractSerializer> m_serializer;

    // Branch, the workspace is based on, if any
    std::string m_branchname;

    // Name of the workspace
    std::string m_name;

    std::shared_ptr<WorkspaceObj> m_workspace;

    std::vector<ObjectId> m_transientOids;
    ObjectId m_nextTransientOid;
};

//TODO: This method could be implemented shorter with less branching and duplication in code.
// This method ensures that the path consist of transient objects.
// This is needed when objects are changed in a workspace, because history must not be changed.
// Method walks through path and detects the first branch into history. It then copies all objects from
// history or creates new objects.
// If "createLeafe" is set, the last element of the path will be hooked correctly into the
// (maybe newly created) parent.
template <typename T>
StatusCode WorkspaceImpl::createPath(const Path &path, std::shared_ptr<T> createLeaf, bool deleteLeaf)
{
    assert(!path.empty());
    Path p = preparePath(path);

    // final path of "transient" objects
    std::vector< std::shared_ptr<Tree> > transientTreePath;

    // current element for iteration (only for trees)
    std::shared_ptr<Tree> current;

    // check if root tree exists
    bool rootMissing = !m_workspace->rollingcommit().has_root() || m_workspace->rollingcommit().root().id().empty() && m_workspace->rollingcommit().root().path().empty();

    // check if root is transient/exclusive
    // if root is not transient and must be copied to edit.
    bool rootNotTransient = (!rootMissing) && (!m_workspace->rollingcommit().root().id().empty() && m_workspace->rollingcommit().root().path().empty());
    //bool rootNotExclusive = m_workspace->transientoidstoorigin().count(m_workspace->rollingcommit().root()) == 0;
    assert(rootMissing || (rootNotTransient == (m_workspace->transientoidstoorigin().count(m_workspace->rollingcommit().root().path()) == 0)));
    // if there is no root directory, workspace must be empty and have nothing transient
    assert(!rootMissing || rootMissing && m_workspace->transientoidstoorigin_size() == 0);

    // step 1) make root exclusive or create new root and hook it into rolling commit.
    // This can not be done in step 2), because the parent is not a tree.

    if(rootMissing || rootNotTransient)
    {
        // create/copy root for workspace

        // ObjectIdentifier/Hash of original object.
        // Commit stores the origin-version of each altered object in transientoidstoorigin (empty if newly created).
        ::std::string originOid;

        if(rootMissing)
        {
            // create new root
            originOid = "";
            current = std::shared_ptr<Tree>(new Tree);
        }
        else
        {
            // copy to make transient
            originOid = m_workspace->rollingcommit().root().id();
            current = m_serializer->getTree(originOid);
        }
        m_workspace->mutable_rollingcommit()->mutable_root()->set_path(m_name + "/");
        m_workspace->mutable_transientoidstoorigin()
                ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(m_workspace->rollingcommit().root().path(), originOid));
        transientTreePath.push_back( current );
    }
    else
    {
        // root exists and is transient; can be altered
        PathInternal path = m_workspace->rollingcommit().root().path();
        current = m_serializer->getTreeTransient(path);
        transientTreePath.push_back( current );
    }

    // step 2) step though trees. Fill "transientTreePath" and finally persist all objects
    // This must be done in two steps:
    //   2.1) construct the objects in memory and update them (fill transientTreePath)
    //   2.2) persist objects and ensure parent/child relationship

    // indicates if leaf was successfully appended in "before" step
    bool leafWasStored = false;
    PathInternal leafPathIntenal;
    forEachPathSegment(p,
    [&](std::string seg, size_t idx, bool isLast)
    {
        if(current == NULL) return false; //< can not go futher
        if(seg.empty()) return false; //< "//" not allowed

        ObjectReference ref = objectReferenceOfChild(current, seg);
        PathInternal pathInternal = m_name + "/" + p.substr(0,idx);

        // For each segment the object might be
        // - not yet existant -> create
        // - already transient -> alter
        // - not yet transient -> copy and alter
        // for each of these cases we must handle the last element special
        // ...which leads to 6 cases:
        // 1+2) no oid yet          -> create tree/entity, put in vector to update and store. (for leaf: create directly)
        // 3+4) transient oids      -> put in vector to update and store (for leaf: create directly)
        // 5+6) non-transient oids  -> put copy in vector to update and store (for leaf: create directly)
        // for deleting we have to 3 more cases, for each case we have to remove the reference to the leaf from its parrent
        // 7)   no oid yet          -> we don't have to do anything at all (not eaven removing of the reference in the parrent)
        // 8)   transient oids      -> we need to delete the leaf and its children
        // 9)   non-transient oids  -> we don't have to do anything more

        assert(ref.id().empty() || ref.path().empty()); // both must not be set at the same time.

        bool segNotExistant = ref.id().empty() && ref.path().empty();
        if(segNotExistant)
        {
            // delete leaf
            if (isLast && deleteLeaf)
            {
                // case 7: we don't need to do anything, since the leaf dosn't exists
            }
            // create new
            else if(!isLast || (isLast && createLeaf == NULL))
            {
                // case 1: append tree
                std::shared_ptr<Tree> tree(new Tree);
                transientTreePath.push_back( tree );
                m_workspace->mutable_transientoidstoorigin()
                        ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(pathInternal, ""));
                current = tree;
            }
            else
            {
                // case 2: create Leaf. This is executed once. Next step is after(...).
                std::pair<StatusCode, PathInternal> pathStatus = storeObject(createLeaf, pathInternal);
                leafPathIntenal = pathStatus.second;
                if(!mapitIsOk(pathStatus.first)) return false;
                m_workspace->mutable_transientoidstoorigin()
                        ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(pathStatus.second, "")); //TODO: history of createLeaf lost, if ther was one

                leafWasStored = true;
            }
        }
        else
        {
            // use or copy existing
            bool segTransient = ref.id().empty() && !ref.path().empty();
            //bool segTransient = m_workspace->transientoidstoorigin().count(oid) != 0; (other way to determine if seg is transient, instead this is ensured in assertion below)
            assert(segTransient == (m_workspace->transientoidstoorigin().count(ref.path()) != 0)); //< if this happens, commit did not track objects correctly!
            if(segTransient)
            {
                // delete leaf
                if (isLast && deleteLeaf)
                {
                    // case 8: remove this (and in case of a tree its children)
                    deleteObject<T>( p.substr(0,idx) );
                }
                // use existing
                else if(!isLast || (isLast && createLeaf == NULL))
                {
                    // case 3: put tree in vector and do nothing
                    std::shared_ptr<Tree> tree(m_serializer->getTreeTransient(ref.path()));
                    if(tree == NULL)
                    {
                        log_error("Segment of path was not a tree or path was wrong");
                        return false;
                    }
                    transientTreePath.push_back( tree );
                    current = tree;
                }
                else
                {
                    // case 4: overwrite Leaf. This is executed once. Next step is after(...).
                    // Note: here might be two transient ids in the pair
                    m_workspace->mutable_transientoidstoorigin()
                            ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(pathInternal, "")); //TODO: history of createLeaf lost, if ther was one
                    std::pair<StatusCode, PathInternal> statusPath = storeObject(createLeaf, pathInternal);
                    if(!mapitIsOk(statusPath.first)) return false;
                    leafPathIntenal = statusPath.second;
                    leafWasStored = true;
                }
            }
            else
            {
                // copy
                assert(!ref.id().empty() && ref.path().empty());
                // delete leaf
                if (isLast && deleteLeaf)
                {
                    // case 9: only the reference of the parrent needs to be removed
                }
                else if(!isLast || (isLast && createLeaf == NULL))
                {
                    // case 5: copy existing tree under transient oid
                    std::shared_ptr<Tree> tree(m_serializer->getTree(ref.id()));
                    if(tree == NULL)
                    {
                        log_error("Segment of path was not a tree or corrupt oids");
                        return false;
                    }
                    m_workspace->mutable_transientoidstoorigin()
                            ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(pathInternal, ref.id()));
                    transientTreePath.push_back( tree );
                    current = tree;
                }
                else
                {
                    // case 6: overwrite Leaf with copy. This is executed once. Next step is after(...).
                    m_workspace->mutable_transientoidstoorigin()
                            ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(pathInternal, ref.id()));
                    //createLeaf->set_id(nextOid);
                    std::pair<StatusCode, PathInternal> status_path = storeObject(createLeaf, pathInternal);
                    if(!mapitIsOk(status_path.first)) return false;
                    leafPathIntenal = status_path.second;
                    leafWasStored = true;
                }
            }

        }
        return true; // continue thru path
    },
    [&](std::string seg, size_t idx, bool isLast)
    {
        // store object
        PathInternal pathInternal;
        if (isLast && deleteLeaf)
        {
            // this is the leaf we just deleted, we don't need to do anything now, but need to remove its reference at the bottom
        }
        else if(leafWasStored && isLast) // leaf already created in "before"
        {
            assert(!leafPathIntenal.empty());
            pathInternal = leafPathIntenal;
        }
        else
        {
            current = transientTreePath.back();
            transientTreePath.pop_back();
            pathInternal = m_name + "/" + p.substr(0, idx);
            std::pair<StatusCode, PathInternal> status_path = m_serializer->storeTreeTransient(current, pathInternal);
            pathInternal = status_path.second;
            if(!mapitIsOk(status_path.first)) { assert(false); return false;} // must never happen. leads to inconsistent data. TODO: rollback
            if(transientTreePath.empty())
            {
                assert(idx == 0);
                return false;
            }
        }

        // update parent child relationship
        std::shared_ptr<Tree> parent = transientTreePath.back();
        assert(parent);
        if (isLast && deleteLeaf)
        {
            // this is the leaf we just deleted, now we have to remove its reference
            parent->mutable_refs()->erase(seg);
        }
        else
        {
            ObjectReference oref;
            oref.set_path(pathInternal);
            //oref.set_lastchange(QDateTime::currentDateTime().toMSecsSinceEpoch()); //< breaks hashing
            parent->mutable_refs()->erase(seg);
            parent->mutable_refs()
                    ->insert( ::google::protobuf::MapPair< ::std::string, ::mapit::msgs::ObjectReference>( seg, oref));
        }
        return true;
    });

    // store/create root
    std::shared_ptr<Tree> obj = transientTreePath.back();
    transientTreePath.pop_back();
    std::pair<StatusCode, PathInternal> status_path = m_serializer->storeTreeTransient(obj, m_name + "/");
    if(!mapitIsOk(status_path.first)) return status_path.first;

    // update workspace commit
    StatusCode s = m_serializer->storeWorkspaceCommit(m_workspace, m_name);
    return s;
}

}
#endif
