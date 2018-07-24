/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "workspaceimpl.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#include <string>
#include <sstream>
#include <algorithm>
#include <mapit/operators/module.h>
#include "operationenvironmentimpl.h"
#include <mapit/serialization/entitydatalibrarymanager.h>
#include <mapit/depthfirstsearch.h>
#include <mapit/serialization/operatorlibrarymanager.h>

namespace mapit
{
typedef ModuleInfo* (*GetModuleInfo)();

WorkspaceImpl::WorkspaceImpl(std::shared_ptr<AbstractSerializer> serializer, std::shared_ptr<WorkspaceObj> workspaceCommit, std::string name, const std::string branchname)
    :m_serializer(serializer),
     m_branchname( branchname ),
     m_name(name),
     m_workspace(workspaceCommit),
     m_nextTransientOid("0")
{

//    if(m_serializer->isWorkspace(workspaceCommit))
//    {
//        m_checkoutId = commitOrCheckoutId;
//    }
//    else if(m_serializer->isCommit(commitOrCheckoutId))
//    {
//        std::shared_ptr<Commit> co(new Commit());
//        co->add_parentcommitids(commitOrCheckoutId);
//        m_checkoutId = m_serializer->createworkspaceCommit( co );
//    }
}

//WorkspaceImpl::WorkspaceImpl(AbstractSerializer *serializer, const std::shared_ptr<Branch> &branch)
//    :m_serializer(serializer),
//      m_branch( branch )
//{
////    if(m_serializer->isWorkspace(branch->commitid()))
////    {
////        m_checkoutId = branch->commitid();
////    }
////    else
////    {
////        std::shared_ptr<Commit> co(new Commit());
////        if(m_serializer->isCommit(branch->commitid()))
////        {
////            co->add_parentcommitids(branch->commitid());
////        }
////        else
////        {
////            log_info("Initial Commit created, branch: " + branch->name());
////        }
////        m_checkoutId = m_serializer->createworkspaceCommit( co );
////    }
//}

bool WorkspaceImpl::isInConflictMode()
{
    return false;
}

std::vector<std::shared_ptr<Conflict> > WorkspaceImpl::getPendingConflicts()
{
    return std::vector<std::shared_ptr<Conflict> >();
}

std::shared_ptr<Commit>
WorkspaceImpl::getRollingcommit()
{
    return std::make_shared<Commit>( m_workspace->rollingcommit() );
}

std::shared_ptr<Tree> WorkspaceImpl::getRoot()
{
    return this->getTree(m_workspace->rollingcommit().root());
}

std::shared_ptr<Tree> WorkspaceImpl::getTree(const Path &path)
{
    assert(!path.empty());
    Path p(preparePath(path));
    if(p.empty()) return nullptr;

    // Try to get a transient tree
    std::shared_ptr<Tree> tree = m_serializer->getTreeTransient(m_name + "/" + p);
    if(tree) return tree;

    // if tree is not transient, get from repo
    ObjectReference ref = objectReferenceForPath(p);
    if(ref.id().empty()) ref.set_id( path );
    return m_serializer->getTree(ref.id());
}

std::shared_ptr<Entity> WorkspaceImpl::getEntity(const Path &path)
{
    assert(!path.empty());
    Path p(preparePath(path));
    if(p.empty()) return nullptr;

    // Try to get a transient entity
    std::shared_ptr<Entity> ent = m_serializer->getEntityTransient(m_name + "/" + p);
    if(ent) return ent;

    // if entity is not transient, get from repo
    ObjectReference ref = objectReferenceForPath(p);
    //assert(!ref.id().empty() && ref.path().empty());
    if(ref.id().empty()) ref.set_id( path );
    return m_serializer->getEntity(ref.id());
}

MessageType
WorkspaceImpl::typeOfObject(const Path &path)
{
    assert(!path.empty());
    Path p(preparePath(path));
    if(p.empty()) return MessageEmpty;

    // Try to get a transient object
    MessageType type = m_serializer->typeOfObjectTransient(m_name + "/" + p);
    if(type != MessageEmpty) return type;

    // if object is not transient, get from repo
    ObjectReference ref = objectReferenceForPath(p);
    if(ref.id().empty()) ref.set_id( path );
    return m_serializer->typeOfObject(ref.id());
}

std::shared_ptr<Tree> WorkspaceImpl::getTreeConflict(const ObjectId &objectId)
{
    return nullptr;
}

std::shared_ptr<Entity> WorkspaceImpl::getEntityConflict(const ObjectId &objectId)
{
    return nullptr;
}

mapit::StatusCode WorkspaceImpl::storeOperationDesc_(const OperationDescription &desc, bool restorable)
{
    OperationDescription* addedDesc = m_workspace->mutable_rollingcommit()->add_ops();
    addedDesc->mutable_operator_()->set_operatorname( desc.operator_().operatorname() );
    addedDesc->mutable_operator_()->set_operatorversion( desc.operator_().operatorversion() );
    addedDesc->mutable_operator_()->set_restorable( restorable );
    addedDesc->set_params( desc.params() );
    m_serializer->storeWorkspaceCommit(m_workspace, m_name);

    return MAPIT_STATUS_OK;
}

OperationResult WorkspaceImpl::doOperation(const OperationDescription &desc)
{
    OperationResult result = OperatorLibraryManager::doOperation(desc, this);
    // when operation successfull, add to commit
    if ( mapitIsOk(result.first) ) {
        storeOperationDesc_(desc, result.second.operator_().restorable());
    }
    return result;
}

OperationResult WorkspaceImpl::doUntraceableOperation(const OperationDescription &desc, std::function<mapit::StatusCode(mapit::OperationEnvironment*)> operate)
{
    OperationEnvironmentImpl env(desc);
    env.setWorkspace( this );
    env.setOutputDescription(desc, false);
    mapit::StatusCode result = operate( &env );
    return OperationResult(result, env.outputDescription());
}

std::shared_ptr<AbstractEntitydata> WorkspaceImpl::getEntitydataReadOnly(const Path &path)
{
    Path p(preparePathFilename(path));
    if(p.empty()) return nullptr;

    std::shared_ptr<Entity> ent = m_serializer->getEntityTransient(m_name + "/" + p );
    if( ent != nullptr )
    {
        return EntityDataLibraryManager::getEntitydataFromProvider(ent->type(), m_serializer->getStreamProviderTransient(m_name + "/" + p, true, false), true);
    }
    else
    {
        // if object is not transient, get from repo
        ObjectReference ref = objectReferenceForPath(p);
        if(ref.id().empty()) return nullptr;

        std::shared_ptr<Entity> ent = m_serializer->getEntity( ref.id() );
        if( ent == nullptr )
        {
            log_warn("Entity not found." + path);
            return nullptr;
        }
        return EntityDataLibraryManager::getEntitydataFromProvider(ent->type(), m_serializer->getStreamProvider(ref.id(), true), true);
    }
}

std::shared_ptr<AbstractEntitydata> WorkspaceImpl::getEntitydataReadOnlyConflict(const ObjectId &entityId)
{
    std::shared_ptr<Entity> ent = m_serializer->getEntity( entityId );
    if( ent == nullptr )
    {
        log_warn("Entity not found." + entityId);
        return nullptr;
    }
    assert( ent );
    return EntityDataLibraryManager::getEntitydataFromProvider(ent->type(), m_serializer->getStreamProvider(entityId, true), true);
}

std::shared_ptr<AbstractEntitydata> WorkspaceImpl::getEntitydataForReadWrite(const Path &path)
{
    // Only check transient, because writing is not allowed for already commited objects.
    Path p(preparePathFilename(path));
    if(p.empty()) return nullptr;

    std::shared_ptr<Entity> ent = m_serializer->getEntityTransient(m_name + "/" + p );
    if( ent == nullptr )
    {
        log_error("Entity not found." + path);
        return nullptr;
    }
    assert( ent );
    return EntityDataLibraryManager::getEntitydataFromProvider(ent->type(), m_serializer->getStreamProviderTransient(m_name + "/" + p, true, true), true);
}

StatusCode WorkspaceImpl::storeTree(const Path &path, std::shared_ptr<Tree> tree)
{
    return createPath(path, tree);
}

StatusCode WorkspaceImpl::storeEntity(const Path &path, std::shared_ptr<Entity> entity)
{
    return createPath(path, entity);
}

StatusCode WorkspaceImpl::deleteTree(const Path &path)
{
    assert(!path.empty());
    Path p(preparePath(path));
    if (p.empty()) {
        return MAPIT_STATUS_ERROR;
    }

    createPath<Tree>(path, nullptr, true);
    return MAPIT_STATUS_OK;
}

StatusCode WorkspaceImpl::deleteEntity(const Path &path)
{
    assert(!path.empty());
    Path p(preparePath(path));
    if (p.empty()) {
        return MAPIT_STATUS_ERROR;
    }

    createPath<Entity>(path, nullptr, true);
    return MAPIT_STATUS_OK;
}

void WorkspaceImpl::setConflictSolved(const Path &path, const ObjectId &oid)
{
    //TODO
}

StatusCode WorkspaceImpl::depthFirstSearch(  std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree
                                          , std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree
                                          , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity
                                          , std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity)
{
    return mapit::depthFirstSearchWorkspace(this, beforeTree, afterTree, beforeEntity, afterEntity);
}

StatusCode WorkspaceImpl::depthFirstSearch(  const Path& path
                                          , std::function<bool(std::shared_ptr<mapit::msgs::Tree>, const mapit::msgs::ObjectReference&, const Path&)> beforeTree
                                          , std::function<bool(std::shared_ptr<mapit::msgs::Tree>, const mapit::msgs::ObjectReference&, const Path&)> afterTree
                                          , std::function<bool(std::shared_ptr<mapit::msgs::Entity>, const mapit::msgs::ObjectReference&, const Path&)> beforeEntity
                                          , std::function<bool(std::shared_ptr<mapit::msgs::Entity>, const mapit::msgs::ObjectReference&, const Path&)> afterEntity)
{
    return mapit::depthFirstSearchWorkspace(this, path, beforeTree, afterTree, beforeEntity, afterEntity);
}

std::shared_ptr<WorkspaceObj> WorkspaceImpl::getWorkspaceObj()
{
    return m_workspace;
}

const std::string &WorkspaceImpl::getName()
{
    return m_name;
}

const std::string &WorkspaceImpl::getBranchName() const
{
    return m_branchname;
}

std::shared_ptr<Tree> WorkspaceImpl::getTree(const ObjectReference &ref)
{
    if ( ! ref.path().empty()) {
        return m_serializer->getTreeTransient(ref.path());
    } else if( ! ref.id().empty()) {
        return m_serializer->getTree(ref.id());
    } else {
        return std::shared_ptr<Tree>(nullptr);
    }
    //assert(false);
}

std::shared_ptr<Entity> WorkspaceImpl::getEntity(const ObjectReference &ref)
{
    if(!ref.path().empty())
    {
        return m_serializer->getEntityTransient(ref.path());
    }
    else if (!ref.id().empty())
    {
        return m_serializer->getEntity(ref.id());
    }
    assert(false);
}

MessageType WorkspaceImpl::typeOfObject(const ObjectReference &ref)
{
    if(!ref.path().empty())
    {
        return m_serializer->typeOfObjectTransient(ref.path());
    }
    else if (!ref.id().empty())
    {
        return m_serializer->typeOfObject(ref.id());
    }
    assert(false);
}

ObjectReference WorkspaceImpl::objectReferenceOfChild(std::shared_ptr<Tree> tree, const ::std::string &name)
{
    assert(tree != NULL);
    assert(!name.empty());
    const ::google::protobuf::Map< ::std::string, ::mapit::msgs::ObjectReference > &refs = tree->refs();
    ::google::protobuf::Map< ::std::string, ::mapit::msgs::ObjectReference >::const_iterator iter(refs.cbegin());
    while(iter != refs.cend())
    {
        const ::std::string &refname(iter->first);
        if(refname == name)
        {
            return iter->second;
        }
        iter++;
    }
    return ObjectReference();
}

ObjectReference WorkspaceImpl::objectReferenceForPath(const Path &path)
{
    assert(!path.empty());
    Path p = preparePath(path);
    ObjectReference ref(m_workspace->rollingcommit().root());
    if(ref.id().empty() && ref.path().empty()) return ObjectReference();
    std::shared_ptr<Tree> current = this->getTree(ref);
    forEachPathSegment(p,
    [&](std::string seg, size_t idx, bool isLast)
    {
        if(current == NULL) return false; // can not go futher
        if(seg.empty()) return false; // "//" not allowed
        ref = objectReferenceOfChild(current, seg);
        if(ref.id().empty() && ref.path().empty()) return false; // path invalid
        if(isLast) return false; // we are done
        current = this->getTree(ref);
        return true; // continue thru path
    },
    [](std::string seg, size_t idx, bool isLast)
    {
        assert(false);
        return false;
    });
    return ref;
}

Path WorkspaceImpl::preparePath(const Path &path)
{
    // path p has no beginning / and always trailing /
    // also removed double //
    // root "/" remains "/"
    Path p = path;
    assert(!p.empty());
    while(p[0] == '/')
    {
        if(p.length() == 1) return p;
        p = p.substr(1);
    }
    if(p.length() != 0 && p[p.length()-1] != '/')
    {
        p += "/";
    }

    // search and replace double // until there are no more of it
    size_t f = p.find("//");
    while (std::string::npos != f) {
        p.replace(f, std::string("//").length(), "/");
        f = p.find("//");
    }

    return p;
}

Path WorkspaceImpl::preparePathFilename(const Path &path)
{
    // path p has no beginning / and no trailing /
    Path p = path;
    while(p[0] == '/')
    {
        p = p.substr(1);
    }
    while(p.length() != 0 && p[p.length()-1] == '/')
    {
        p = p.substr(0, p.length()-1);
    }
    return p;
}

bool WorkspaceImpl::forEachPathSegment(const Path &path
                        , std::function<bool(std::string, size_t, bool)> before
                        , std::function<bool(std::string, size_t, bool)> after
                        , const int start)
{
    assert(!path.empty());
    size_t nextSlash = path.find_first_of('/', start);
    std::string segment(path.substr(start, nextSlash-start));
    bool isLast = nextSlash == std::string::npos || nextSlash == path.length()-1;
    if(!before(segment, nextSlash+1, isLast))
    {
        return false;
    }
    if(!isLast)
    {
        if(!forEachPathSegment(path, before, after, nextSlash+1))
        {
            return false;
        }
    }
    if(!after(segment, nextSlash+1, isLast))
    {
        return false;
    }
    return true;
}

// TODO: Object-classes are seperated (entity, tree, ...) and are here unified again.
// On the lowest level they are now unified. TODO: provide unified interface to them (template).
//template <>
//std::pair<StatusCode, ObjectId> WorkspaceImpl::createObject<Tree>(std::shared_ptr<Tree> leafObject, const Path &path)
//{
//    return m_serializer->storeTreeTransient(leafObject, path);
//}
//TODO: maybe abandon "create" for entity/tree
//template <>
//std::pair<StatusCode, ObjectId> WorkspaceImpl::createObject<Entity>(std::shared_ptr<Entity> leafObject, const Path &path)
//{
//    return m_serializer->storeEntityTransient(leafObject, path);
//}

template <>
std::pair<StatusCode, PathInternal> WorkspaceImpl::storeObject<Tree>(std::shared_ptr<Tree> leafObject, const PathInternal &pathInternal)
{
    return m_serializer->storeTreeTransient(leafObject, pathInternal);
}

template <>
std::pair<StatusCode, PathInternal> WorkspaceImpl::storeObject<Entity>(std::shared_ptr<Entity> leafObject, const PathInternal &pathInternal)
{
    PathInternal real_id = pathInternal.substr(0, pathInternal.size() - 1 );
    return m_serializer->storeEntityTransient(leafObject, real_id);
}

template <>
StatusCode WorkspaceImpl::deleteObject<Tree>(const Path& path)
{
    StatusCode status_search = MAPIT_STATUS_OK;
    ObjectReference nullRef;
    mapit::depthFirstSearchWorkspace(
                this,
                getTree(path), // we just need to delete trees that are actualy transient
                nullRef,
                path,
                depthFirstSearchWorkspaceAll(Tree),
                [&](std::shared_ptr<mapit::msgs::Tree> obj, const ObjectReference& ref, const mapit::Path &pathInt)
                {
                    assert(!pathInt.empty());
                    Path p(preparePath(pathInt));
                    if (p.empty()) {
                        status_search = MAPIT_STATUS_ERROR;
                        return false;
                    }

                    StatusCode s =  m_serializer->removeTreeTransient(m_name + "/" + p);
                    if ( ! mapitIsOk(s) ) {
                        status_search = s;
                        return false;
                    }

                    return true;
               },
                depthFirstSearchWorkspaceAll(Entity),
                [&](std::shared_ptr<mapit::msgs::Entity> obj, const ObjectReference& ref, const mapit::Path &pathInt)
                {
                    StatusCode s = deleteEntity(pathInt);
                    if ( ! mapitIsOk(s) ) {
                        status_search = s;
                        return false;
                    }

                    return true;
                }
            );

    return status_search;
}

template <>
StatusCode WorkspaceImpl::deleteObject<Entity>(const Path& path)
{
    PathInternal real_id = m_name + "/" + path.substr(0, path.size() - 1 );
    return m_serializer->removeEntityTransient(real_id);
}

//StatusCode WorkspaceImpl::depthFirstSearch(std::shared_ptr<Entity> obj, const ObjectId& oid, const Path &path,
//                                                  std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> afterCommit,
//                                                  std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> afterTree,
//                                                  std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> afterEntity)
//{
//    assert(obj != NULL);
//    if(!beforeEntity(obj, oid, path))
//    {
//        afterEntity(obj, oid, path);
//        return MAPIT_STATUS_OK;
//    }
//    //TODO: Entitydata!
//    if(!afterEntity(obj, oid, path)) return MAPIT_STATUS_OK;
//    return MAPIT_STATUS_OK;
//}

//StatusCode WorkspaceImpl::depthFirstSearch(std::shared_ptr<Tree> obj, const ObjectId& oid, const Path &path,
//                                                std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> afterCommit,
//                                                std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> afterTree,
//                                                std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> afterEntity)
//{
//    assert(obj != NULL);
//    if(!beforeTree(obj, oid, path))
//    {
//        afterTree(obj, oid, path);
//        return MAPIT_STATUS_OK;
//    }
//    ::google::protobuf::Map< ::std::string, ::mapit::ObjectReference > &refs = *obj->mutable_refs();
//    ::google::protobuf::Map< ::std::string, ::mapit::ObjectReference >::iterator iter(refs.begin());
//    while(iter != refs.cend())
//    {
//        const ObjectId &childoid = iter->second.id();
//        const Path &childpath = path + "/" + iter->first;
//        MessageType t = this->typeOfObject(childoid);
//        if(t == MessageType::MessageCommit)
//        {
//            std::shared_ptr<Commit> commit(this->getCommit(childoid));
//            StatusCode s = depthFirstSearch(commit, childoid, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
//            if(!mapitIsOk(s)) return s;
//        }
//        else if(t == MessageType::MessageTree)
//        {
//            std::shared_ptr<Tree> tree(this->getTree(childoid));
//            StatusCode s = depthFirstSearch(tree, childoid, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
//            if(!mapitIsOk(s)) return s;
//        }
//        else if(t == MessageType::MessageEntity)
//        {
//            std::shared_ptr<Entity> entity(this->getEntity(childoid));
//            StatusCode s = depthFirstSearch(entity, childoid, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
//            if(!mapitIsOk(s)) return s;
//        }
//        else
//        {
//            log_error("Unsupported type during depth search " + iter->first);
//        }
//        iter++;
//    }
//    if(!afterTree(obj, oid, path)) return MAPIT_STATUS_OK; //TODO: what is happening here?
//    return MAPIT_STATUS_OK;
//}

//StatusCode WorkspaceImpl::depthFirstSearch(std::shared_ptr<Commit> obj, const ObjectId& oid, const Path &path,
//                                                  std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> afterCommit,
//                                                  std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> afterTree,
//                                                  std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> afterEntity)
//{
//    assert(obj != NULL);
//    if(!beforeCommit(obj, oid, path))
//    {
//        afterCommit(obj, oid, path);
//        return MAPIT_STATUS_OK;
//    }
//    std::shared_ptr<Tree> tree(this->getTree(obj->root()));
//    if( !obj->root().empty() )
//    {
//        StatusCode s = depthFirstSearch(tree, obj->root(), "", beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
//        if(!mapitIsOk(s)) return s;
//    }
//    if(!afterCommit(obj, oid, path)) return MAPIT_STATUS_OK;
//    return MAPIT_STATUS_OK;
//}

std::shared_ptr<mapit::msgs::Branch> mapit::WorkspaceImpl::getParentBranch()
{
    std::shared_ptr<mapit::msgs::Branch> branch;
    if(!m_branchname.empty())
    {
        branch = m_serializer->getBranch(m_branchname);
    }
    return branch;
}

std::vector<CommitId> WorkspaceImpl::getParentCommitIds()
{
    std::vector<CommitId> ret;
    for(int i=0 ; i<m_workspace->rollingcommit().parentcommitids_size() ; ++i)
    {
        ret.push_back(m_workspace->rollingcommit().parentcommitids(i));
    }
    return ret;
}

}
