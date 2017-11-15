#include "checkoutimpl.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#include <string>
#include <sstream>
#include <algorithm>
#include <upns/operators/module.h>
#include "operationenvironmentimpl.h"
#include <upns/serialization/entitydatalibrarymanager.h>
#include <upns/depthfirstsearch.h>
#include <upns/serialization/operatorlibrarymanager.h>

namespace upns
{
typedef ModuleInfo* (*GetModuleInfo)();

CheckoutImpl::CheckoutImpl(std::shared_ptr<AbstractSerializer> serializer, std::shared_ptr<CheckoutObj>  checkoutCommit, std::string name, const std::string branchname)
    :m_serializer(serializer),
     m_branchname( branchname ),
     m_name(name),
     m_checkout(checkoutCommit),
     m_nextTransientOid("0")
{

//    if(m_serializer->isCheckout(checkoutCommit))
//    {
//        m_checkoutId = commitOrCheckoutId;
//    }
//    else if(m_serializer->isCommit(commitOrCheckoutId))
//    {
//        std::shared_ptr<Commit> co(new Commit());
//        co->add_parentcommitids(commitOrCheckoutId);
//        m_checkoutId = m_serializer->createCheckoutCommit( co );
//    }
}

//CheckoutImpl::CheckoutImpl(AbstractSerializer *serializer, const std::shared_ptr<Branch> &branch)
//    :m_serializer(serializer),
//      m_branch( branch )
//{
////    if(m_serializer->isCheckout(branch->commitid()))
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
////        m_checkoutId = m_serializer->createCheckoutCommit( co );
////    }
//}

bool CheckoutImpl::isInConflictMode()
{
    return false;
}

std::vector<std::shared_ptr<Conflict> > CheckoutImpl::getPendingConflicts()
{
    return std::vector<std::shared_ptr<Conflict> >();
}

std::shared_ptr<Tree> CheckoutImpl::getRoot()
{
    return this->getTree(m_checkout->rollingcommit().root());
}

std::shared_ptr<Tree> CheckoutImpl::getTree(const Path &path)
{
    Path p(preparePath(path));
    if(p.empty()) return nullptr;
//    if(p.compare(0, m_name.size(), m_name) == 0) // TODO this does not work if checkoutname equals map name at the beginning, e.g. co=test map=testmap/, results in testmap/ being changed to map/
//    {
//        p = p.substr(m_name.size(), p.length()-m_name.size());
//        if (p.compare("/") == 0) { // TODO is this the correct workaround???
//            p = "";
//        }
//    }
    // Try to get a transient tree
    std::shared_ptr<Tree> tree = m_serializer->getTreeTransient(m_name + "/" + p);
    if(tree) return tree;

    // if tree is not transient, get from repo
    ObjectReference ref = objectReferenceForPath(p);
    //assert(!ref.id().empty() && ref.path().empty());
    if(ref.id().empty()) return nullptr;
    return m_serializer->getTree(ref.id());
}

std::shared_ptr<Entity> CheckoutImpl::getEntity(const Path &path)
{
    Path p(preparePath(path));
    if(p.empty()) return nullptr;
//    if(p.compare(0, m_name.size(), m_name) == 0)
//    {
//        p = p.substr(m_name.size(), p.length()-m_name.size());
//    }
    // Try to get a transient entity
    std::shared_ptr<Entity> ent = m_serializer->getEntityTransient(m_name + "/" + p);
    if(ent) return ent;

    // if entity is not transient, get from repo
    ObjectReference ref = objectReferenceForPath(p);
    //assert(!ref.id().empty() && ref.path().empty());
    if(ref.id().empty()) return nullptr;
    return m_serializer->getEntity(ref.id());
}

MessageType
CheckoutImpl::typeOfObject(const Path &path)
{
    Path p(preparePath(path));
    if(p.empty()) return MessageEmpty;
//    if(p.compare(0, m_name.size(), m_name) == 0) // TODO should I fix it
//    {
//        p = p.substr(m_name.size(), p.length()-m_name.size());
//        if (p.compare("/") == 0) { // TODO is this the correct workaround???
//            p = "";
//        }
//    }
    // Try to get a transient object
    MessageType type = m_serializer->typeOfObjectTransient(m_name + "/" + p);
    if(type != MessageEmpty) return type;

    // if object is not transient, get from repo
    ObjectReference ref = objectReferenceForPath(p);
    //assert(!ref.id().empty() && ref.path().empty());
    if(ref.id().empty()) return MessageEmpty;
    return m_serializer->typeOfObject(ref.id());
}

std::shared_ptr<Tree> CheckoutImpl::getTreeConflict(const ObjectId &objectId)
{
    return NULL;
}

std::shared_ptr<Entity> CheckoutImpl::getEntityConflict(const ObjectId &objectId)
{
    return NULL;
}

OperationResult CheckoutImpl::doOperation(const OperationDescription &desc)
{
    return OperatorLibraryManager::doOperation(desc, this);
}

OperationResult CheckoutImpl::doUntraceableOperation(const OperationDescription &desc, std::function<upns::StatusCode(upns::OperationEnvironment*)> operate)
{
    OperationEnvironmentImpl env(desc);
    env.setCheckout( this );
    upns::StatusCode result = operate( &env );
    return OperationResult(result, env.outputDescription());
}

std::shared_ptr<AbstractEntitydata> CheckoutImpl::getEntitydataReadOnly(const Path &path)
{
    Path p(m_name + "/" + preparePathFilename(path));
    std::shared_ptr<Entity> ent = m_serializer->getEntityTransient( p );
    if( ent == NULL )
    {
        log_warn("Entity not found." + path);
        return NULL;
    }
    assert( ent );
    return EntityDataLibraryManager::getEntitydataFromProvider(ent->type(), m_serializer->getStreamProviderTransient(p, true, false), true);
}

std::shared_ptr<AbstractEntitydata> CheckoutImpl::getEntitydataReadOnlyConflict(const ObjectId &entityId)
{
    std::shared_ptr<Entity> ent = m_serializer->getEntity( entityId );
    if( ent == NULL )
    {
        log_warn("Entity not found." + entityId);
        return NULL;
    }
    assert( ent );
    return EntityDataLibraryManager::getEntitydataFromProvider(ent->type(), m_serializer->getStreamProvider(entityId, true), true);
}

std::shared_ptr<AbstractEntitydata> CheckoutImpl::getEntitydataForReadWrite(const Path &path)
{
    Path p(m_name + "/" + preparePathFilename(path));
    std::shared_ptr<Entity> ent = m_serializer->getEntityTransient( p );
    if( ent == NULL )
    {
        log_error("Entity not found." + path);
        return NULL;
    }
    assert( ent );
    return EntityDataLibraryManager::getEntitydataFromProvider(ent->type(), m_serializer->getStreamProviderTransient(p, true, true), true);
}

bool CheckoutImpl::checkEntitydata(const Path &path)
{
    Path p(m_name + "/" + preparePathFilename(path));
    std::shared_ptr<Entity> ent = m_serializer->getEntityTransient( p );
    if( ent == NULL )
    {
        return false;
    }
    return true;
}

StatusCode CheckoutImpl::storeTree(const Path &path, std::shared_ptr<Tree> tree)
{
    return createPath(path, tree);
}

StatusCode CheckoutImpl::storeEntity(const Path &path, std::shared_ptr<Entity> entity)
{
    return createPath(path, entity);
}

void CheckoutImpl::setConflictSolved(const Path &path, const ObjectId &oid)
{
    //TODO
}

StatusCode CheckoutImpl::depthFirstSearch(std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> afterCommit,
                                          std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree,
                                          std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity)
{
    std::shared_ptr<Commit> rootCommit(new Commit(m_checkout->rollingcommit()));
    ObjectReference nullRef;
    StatusCode s = upns::depthFirstSearch(this, rootCommit, nullRef, "", beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
    *m_checkout->mutable_rollingcommit() = *rootCommit;
    return s;
}

std::shared_ptr<CheckoutObj> CheckoutImpl::getCheckoutObj()
{
    return m_checkout;
}

const std::string &CheckoutImpl::getName() const
{
    return m_name;
}

std::shared_ptr<Tree> CheckoutImpl::getTree(const ObjectReference &ref)
{
    if(!ref.id().empty())
    {
        return m_serializer->getTree(ref.id());
    }
    else if (!ref.path().empty())
    {
        return m_serializer->getTreeTransient(ref.path());
    }
    return std::shared_ptr<Tree>(nullptr);
    //assert(false);
}

std::shared_ptr<Entity> CheckoutImpl::getEntity(const ObjectReference &ref)
{
    if(!ref.id().empty())
    {
        return m_serializer->getEntity(ref.id());
    }
    else if (!ref.path().empty())
    {
        return m_serializer->getEntityTransient(ref.path());
    }
    assert(false);
}

MessageType CheckoutImpl::typeOfObject(const ObjectReference &ref)
{
    if(!ref.id().empty())
    {
        return m_serializer->typeOfObject(ref.id());
    }
    else if (!ref.path().empty())
    {
        return m_serializer->typeOfObjectTransient(ref.path());
    }
    assert(false);
}

ObjectReference CheckoutImpl::objectReferenceOfChild(std::shared_ptr<Tree> tree, const ::std::string &name)
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

ObjectReference CheckoutImpl::objectReferenceForPath(const Path &path)
{
    assert(!path.empty());
    Path p = preparePath(path);
    ObjectReference ref(m_checkout->rollingcommit().root());
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

Path CheckoutImpl::preparePath(const Path &path)
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

Path CheckoutImpl::preparePathFilename(const Path &path)
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

bool CheckoutImpl::forEachPathSegment(const Path &path,
                                      std::function<bool (std::string, size_t, bool)> before, std::function<bool (std::string, size_t, bool)> after, const int start)
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
//std::pair<StatusCode, ObjectId> CheckoutImpl::createObject<Tree>(std::shared_ptr<Tree> leafObject, const Path &path)
//{
//    return m_serializer->storeTreeTransient(leafObject, path);
//}
//TODO: maybe abandon "create" for entity/tree
//template <>
//std::pair<StatusCode, ObjectId> CheckoutImpl::createObject<Entity>(std::shared_ptr<Entity> leafObject, const Path &path)
//{
//    return m_serializer->storeEntityTransient(leafObject, path);
//}

template <>
std::pair<StatusCode, PathInternal> CheckoutImpl::storeObject<Tree>(std::shared_ptr<Tree> leafObject, const PathInternal &pathInternal)
{
    return m_serializer->storeTreeTransient(leafObject, pathInternal);
}

template <>
std::pair<StatusCode, PathInternal> CheckoutImpl::storeObject<Entity>(std::shared_ptr<Entity> leafObject, const PathInternal &pathInternal)
{
    PathInternal real_id = pathInternal.substr(0, pathInternal.size() - 1 );
    return m_serializer->storeEntityTransient(leafObject, real_id);
}

//StatusCode CheckoutImpl::depthFirstSearch(std::shared_ptr<Entity> obj, const ObjectId& oid, const Path &path,
//                                                  std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> afterCommit,
//                                                  std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> afterTree,
//                                                  std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> afterEntity)
//{
//    assert(obj != NULL);
//    if(!beforeEntity(obj, oid, path))
//    {
//        afterEntity(obj, oid, path);
//        return UPNS_STATUS_OK;
//    }
//    //TODO: Entitydata!
//    if(!afterEntity(obj, oid, path)) return UPNS_STATUS_OK;
//    return UPNS_STATUS_OK;
//}

//StatusCode CheckoutImpl::depthFirstSearch(std::shared_ptr<Tree> obj, const ObjectId& oid, const Path &path,
//                                                std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> afterCommit,
//                                                std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> afterTree,
//                                                std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> afterEntity)
//{
//    assert(obj != NULL);
//    if(!beforeTree(obj, oid, path))
//    {
//        afterTree(obj, oid, path);
//        return UPNS_STATUS_OK;
//    }
//    ::google::protobuf::Map< ::std::string, ::upns::ObjectReference > &refs = *obj->mutable_refs();
//    ::google::protobuf::Map< ::std::string, ::upns::ObjectReference >::iterator iter(refs.begin());
//    while(iter != refs.cend())
//    {
//        const ObjectId &childoid = iter->second.id();
//        const Path &childpath = path + "/" + iter->first;
//        MessageType t = this->typeOfObject(childoid);
//        if(t == MessageType::MessageCommit)
//        {
//            std::shared_ptr<Commit> commit(this->getCommit(childoid));
//            StatusCode s = depthFirstSearch(commit, childoid, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
//            if(!upnsIsOk(s)) return s;
//        }
//        else if(t == MessageType::MessageTree)
//        {
//            std::shared_ptr<Tree> tree(this->getTree(childoid));
//            StatusCode s = depthFirstSearch(tree, childoid, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
//            if(!upnsIsOk(s)) return s;
//        }
//        else if(t == MessageType::MessageEntity)
//        {
//            std::shared_ptr<Entity> entity(this->getEntity(childoid));
//            StatusCode s = depthFirstSearch(entity, childoid, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
//            if(!upnsIsOk(s)) return s;
//        }
//        else
//        {
//            log_error("Unsupported type during depth search " + iter->first);
//        }
//        iter++;
//    }
//    if(!afterTree(obj, oid, path)) return UPNS_STATUS_OK; //TODO: what is happening here?
//    return UPNS_STATUS_OK;
//}

//StatusCode CheckoutImpl::depthFirstSearch(std::shared_ptr<Commit> obj, const ObjectId& oid, const Path &path,
//                                                  std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference &)> afterCommit,
//                                                  std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference &)> afterTree,
//                                                  std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference &)> afterEntity)
//{
//    assert(obj != NULL);
//    if(!beforeCommit(obj, oid, path))
//    {
//        afterCommit(obj, oid, path);
//        return UPNS_STATUS_OK;
//    }
//    std::shared_ptr<Tree> tree(this->getTree(obj->root()));
//    if( !obj->root().empty() )
//    {
//        StatusCode s = depthFirstSearch(tree, obj->root(), "", beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
//        if(!upnsIsOk(s)) return s;
//    }
//    if(!afterCommit(obj, oid, path)) return UPNS_STATUS_OK;
//    return UPNS_STATUS_OK;
//}

std::shared_ptr<mapit::msgs::Branch> upns::CheckoutImpl::getParentBranch()
{
    std::shared_ptr<mapit::msgs::Branch> branch;
    if(!m_branchname.empty())
    {
        branch = m_serializer->getBranch(m_branchname);
    }
    return branch;
}

std::vector<CommitId> CheckoutImpl::getParentCommitIds()
{
    std::vector<CommitId> ret;
    for(int i=0 ; i<m_checkout->rollingcommit().parentcommitids_size() ; ++i)
    {
        ret.push_back(m_checkout->rollingcommit().parentcommitids(i));
    }
    return ret;
}

}
