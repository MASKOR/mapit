#include "depthfirstsearch.h"
#include "upns_errorcodes.h"

namespace upns
{

StatusCode depthFirstSearch(Checkout *checkout, upnsSharedPointer<Entity> obj, const ObjectId& oid, const Path &path,
                                                  std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> afterCommit,
                                                  std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> afterTree,
                                                  std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> afterEntity)
{
    assert(obj != NULL);
    if(!beforeEntity(obj, oid, path))
    {
        afterEntity(obj, oid, path);
        return UPNS_STATUS_OK;
    }
    //TODO: Entitydata!
    if(!afterEntity(obj, oid, path)) return UPNS_STATUS_OK;
    return UPNS_STATUS_OK;
}

StatusCode depthFirstSearch(Checkout *checkout, upnsSharedPointer<Tree> obj, const ObjectId& oid, const Path &path,
                                                std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> afterCommit,
                                                std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> afterTree,
                                                std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> afterEntity)
{
    assert(obj != NULL);
    if(!beforeTree(obj, oid, path))
    {
        afterTree(obj, oid, path);
        return UPNS_STATUS_OK;
    }
    ::google::protobuf::Map< ::std::string, ::upns::ObjectReference > &refs = *obj->mutable_refs();
    ::google::protobuf::Map< ::std::string, ::upns::ObjectReference >::iterator iter(refs.begin());
    while(iter != refs.cend())
    {
        const ObjectId &childoid = iter->second.id();
        const Path &childpath = path + "/" + iter->first;
        MessageType t = checkout->typeOfObject(childoid);

        if(t == MessageType::MessageCommit)
        {
            assert(false);
            log_error("Commit found in tree. Commit must be root");
            return UPNS_STATUS_ERR_DB_CORRUPTION;
//            upnsSharedPointer<Commit> commit(checkout->getCommit(childoid));
//            StatusCode s = depthFirstSearch(checkout, commit, childoid, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
//            if(!upnsIsOk(s)) return s;
        }
        else if(t == MessageType::MessageTree)
        {
            upnsSharedPointer<Tree> tree(checkout->getTree(childoid));
            StatusCode s = depthFirstSearch(checkout, tree, childoid, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
            if(!upnsIsOk(s)) return s;
        }
        else if(t == MessageType::MessageEntity)
        {
            upnsSharedPointer<Entity> entity(checkout->getEntity(childoid));
            StatusCode s = depthFirstSearch(checkout, entity, childoid, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
            if(!upnsIsOk(s)) return s;
        }
        else
        {
            log_error("Unsupported type during depth search " + iter->first);
        }
        iter++;
    }
    if(!afterTree(obj, oid, path)) return UPNS_STATUS_OK; //TODO: what is happening here?
    return UPNS_STATUS_OK;
}

StatusCode depthFirstSearch(Checkout *checkout, upnsSharedPointer<Commit> obj, const ObjectId& oid, const Path &path,
                                                  std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> afterCommit,
                                                  std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> afterTree,
                                                  std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> afterEntity)
{
    assert(obj != NULL);
    if(!beforeCommit(obj, oid, path))
    {
        afterCommit(obj, oid, path);
        return UPNS_STATUS_OK;
    }
    upnsSharedPointer<Tree> tree(checkout->getTree(obj->root()));
    if( !obj->root().empty() )
    {
        StatusCode s = depthFirstSearch(checkout, tree, obj->root(), "", beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
        if(!upnsIsOk(s)) return s;
    }
    if(!afterCommit(obj, oid, path)) return UPNS_STATUS_OK;
    return UPNS_STATUS_OK;
}
}
