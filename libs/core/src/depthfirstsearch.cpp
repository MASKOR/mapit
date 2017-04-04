#include <upns/depthfirstsearch.h>
#include <upns/errorcodes.h>

namespace upns
{

StatusCode depthFirstSearch(Checkout *checkout, std::shared_ptr<Entity> obj, const ObjectReference &ref, const Path& path,
                            std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> afterCommit,
                            std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree,
                            std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity)
{
    assert(obj != NULL);
    if(!beforeEntity(obj, ref, path))
    {
        afterEntity(obj, ref, path);
        return UPNS_STATUS_OK;
    }
    //TODO: Entitydata!
    if(!afterEntity(obj, ref, path)) return UPNS_STATUS_OK;
    return UPNS_STATUS_OK;
}

StatusCode depthFirstSearch(Checkout *checkout, std::shared_ptr<Tree> obj, const ObjectReference &ref, const Path& path,
                                                std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> afterCommit,
                                                std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree,
                                                std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity)
{
    assert(obj != NULL);
    if(!beforeTree(obj, ref, path))
    {
        afterTree(obj, ref, path);
        return UPNS_STATUS_OK;
    }
    ::google::protobuf::Map< ::std::string, ::mapit::msgs::ObjectReference > &refs = *obj->mutable_refs();
    ::google::protobuf::Map< ::std::string, ::mapit::msgs::ObjectReference >::iterator iter(refs.begin());
    while(iter != refs.cend())
    {
        const ObjectReference &childref = iter->second;
        const Path &childpath = (path=="/"?"":path) + "/" + iter->first;
        MessageType t = checkout->typeOfObject(childpath);

        if(t == MessageType::MessageCommit)
        {
            assert(false);
            log_error("Commit found in tree. Commit must be root");
            return UPNS_STATUS_ERR_DB_CORRUPTION;
//            std::shared_ptr<Commit> commit(checkout->getCommit(childoid));
//            StatusCode s = depthFirstSearch(checkout, commit, childoid, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
//            if(!upnsIsOk(s)) return s;
        }
        else if(t == MessageType::MessageTree)
        {
            std::shared_ptr<Tree> tree(checkout->getTree(childpath));
            StatusCode s = depthFirstSearch(checkout, tree, childref, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
            if(!upnsIsOk(s)) return s;
        }
        else if(t == MessageType::MessageEntity)
        {
            std::shared_ptr<Entity> entity(checkout->getEntity(childpath));
            StatusCode s = depthFirstSearch(checkout, entity, childref, childpath, beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
            if(!upnsIsOk(s)) return s;
        }
        else
        {
            log_error("Unsupported type during depth search " + iter->first);
        }
        iter++;
    }
    if(!afterTree(obj, ref, path)) return UPNS_STATUS_OK; //TODO: what is happening here?
    return UPNS_STATUS_OK;
}

StatusCode depthFirstSearch(Checkout *checkout, std::shared_ptr<Commit> obj, const ObjectReference &ref, const Path& path,
                            std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> beforeCommit, std::function<bool(std::shared_ptr<Commit>, const ObjectReference&, const Path&)> afterCommit,
                            std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> beforeTree, std::function<bool(std::shared_ptr<Tree>, const ObjectReference&, const Path&)> afterTree,
                            std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> beforeEntity, std::function<bool(std::shared_ptr<Entity>, const ObjectReference&, const Path&)> afterEntity)
{
    assert(obj != NULL);
    if(!beforeCommit(obj, ref, path))
    {
        afterCommit(obj, ref, path);
        return UPNS_STATUS_OK;
    }

    std::shared_ptr<Tree> tree(checkout->getRoot());
    if( tree )
    {
        StatusCode s = depthFirstSearch(checkout, tree, obj->root(), "/", beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
        if(!upnsIsOk(s)) return s;
    }
    if(!afterCommit(obj, ref, path)) return UPNS_STATUS_OK;
    return UPNS_STATUS_OK;
}
}
