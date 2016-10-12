#include "upnszmqrequestercheckout.h"
#include "services_internal.pb.h"
#include "serialization/entitystreammanager.h" //TODO: put in yet another independent project/cmake target. No dependecy to mapmanager required.
#include "serialization/zmqentitydatastreamprovider.h"
#include "operationenvironmentimpl.h"

upns::ZmqRequesterCheckout::ZmqRequesterCheckout(upnsString name, ZmqProtobufNode *node, Checkout *cache)
    :m_checkoutName( name ),
     m_node( node ),
     m_cache( cache )
{
}

bool upns::ZmqRequesterCheckout::isInConflictMode()
{
    //TODO: nyi
    return false;
}

upns::upnsVec<upns::upnsSharedPointer<upns::Conflict> > upns::ZmqRequesterCheckout::getPendingConflicts()
{
    //TODO: nyi
    return upns::upnsVec<upns::upnsSharedPointer<upns::Conflict> >();
}

void upns::ZmqRequesterCheckout::setConflictSolved(const upns::Path &path, const upns::ObjectId &oid)
{
    //TODO: nyi
}

upns::upnsSharedPointer<upns::Tree> upns::ZmqRequesterCheckout::getRoot()
{
    std::unique_ptr<upns::RequestHierarchy> req(new upns::RequestHierarchy);
    req->set_checkout(m_checkoutName);
    m_node->send(std::move(req));
    upns::upnsSharedPointer<upns::ReplyHierarchy> rep(m_node->receive<upns::ReplyHierarchy>());
    upns::upnsSharedPointer<upns::Tree> ret(new upns::Tree);
    for(google::protobuf::Map< ::std::string, ::upns::ReplyHierarchyMap >::const_iterator ch( rep->maps().cbegin() );
        ch != rep->maps().cend();
        ++ch)
    {
        ret->mutable_refs()->insert(::google::protobuf::MapPair< ::std::string, ::upns::ObjectReference>(ch->first, upns::ObjectReference()));
    }
    return ret;
}

upns::upnsSharedPointer<upns::Tree> upns::ZmqRequesterCheckout::getTreeConflict(const upns::ObjectId &objectId)
{
    //TODO: nyi
    return upns::upnsSharedPointer<upns::Tree>();
}

upns::upnsSharedPointer<upns::Entity> upns::ZmqRequesterCheckout::getEntityConflict(const upns::ObjectId &objectId)
{
    //TODO: nyi
    return upns::upnsSharedPointer<upns::Entity>();
}

upns::upnsSharedPointer<upns::Tree> upns::ZmqRequesterCheckout::getTree(const upns::Path &path)
{
    std::unique_ptr<upns::RequestTree> req(new upns::RequestTree);
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    m_node->send(std::move(req));
    upns::upnsSharedPointer<upns::ReplyTree> rep(m_node->receive<upns::ReplyTree>());
    upns::upnsSharedPointer<upns::Tree> ret(rep->release_tree());
    return ret;
}

upns::upnsSharedPointer<upns::Entity> upns::ZmqRequesterCheckout::getEntity(const upns::Path &path)
{
    std::unique_ptr<upns::RequestEntity> req(new upns::RequestEntity);
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    m_node->send(std::move(req));
    upns::upnsSharedPointer<upns::ReplyEntity> rep(m_node->receive<upns::ReplyEntity>());
    upns::upnsSharedPointer<upns::Entity> ret(rep->release_entity());
    return ret;
}

upns::upnsSharedPointer<upns::Branch> upns::ZmqRequesterCheckout::getParentBranch()
{
    //TODO: nyi
    return upns::upnsSharedPointer<upns::Branch>();
}

upns::upnsVec<upns::CommitId> upns::ZmqRequesterCheckout::getParentCommitIds()
{
    //TODO: nyi
    return upns::upnsVec<upns::CommitId>();
}

upns::upnsSharedPointer<upns::AbstractEntityData> upns::ZmqRequesterCheckout::getEntitydataReadOnly(const upns::Path &entityId)
{
    upnsSharedPointer<Entity> e = getEntity(entityId);
    if(!e)
    {
        log_error("Entity could not be queried for entitydata: " + entityId);
        return upns::upnsSharedPointer<upns::AbstractEntityData>(nullptr);
    }
    upns::upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider(new ZmqEntitydataStreamProvider(m_checkoutName, entityId, m_node));
    return EntityStreamManager::getEntityDataFromStreamImpl(e->type(), streamProvider);
}

upns::upnsSharedPointer<upns::AbstractEntityData> upns::ZmqRequesterCheckout::getEntitydataReadOnlyConflict(const upns::ObjectId &entityId)
{

}

upns::StatusCode upns::ZmqRequesterCheckout::depthFirstSearch(std::function<bool (upns::upnsSharedPointer<upns::Commit>, const upns::ObjectId &, const upns::Path &)> beforeCommit, std::function<bool (upns::upnsSharedPointer<upns::Commit>, const upns::ObjectId &, const upns::Path &)> afterCommit, std::function<bool (upns::upnsSharedPointer<upns::Tree>, const upns::ObjectId &, const upns::Path &)> beforeTree, std::function<bool (upns::upnsSharedPointer<upns::Tree>, const upns::ObjectId &, const upns::Path &)> afterTree, std::function<bool (upns::upnsSharedPointer<upns::Entity>, const upns::ObjectId &, const upns::Path &)> beforeEntity, std::function<bool (upns::upnsSharedPointer<upns::Entity>, const upns::ObjectId &, const upns::Path &)> afterEntity)
{

}

upns::OperationResult upns::ZmqRequesterCheckout::doOperation(const upns::OperationDescription &desc)
{
    std::unique_ptr<upns::RequestOperatorExecution> req(new upns::RequestOperatorExecution);
    req->set_checkout(m_checkoutName);
    *req->mutable_param() = desc;
    m_node->send(std::move(req));
    upns::upnsSharedPointer<upns::ReplyOperatorExecution> rep(m_node->receive<upns::ReplyOperatorExecution>());
    upns::OperationResult res;
    res.first = rep->status_code();
    res.second = rep->result();
    if(!rep->error_msg().empty())
    {
        log_error("Remote Operator Executions returned an error: " + rep->error_msg());
    }
    return res;
}

upns::OperationResult upns::ZmqRequesterCheckout::doUntraceableOperation(const upns::OperationDescription &desc, std::function<upns::StatusCode (upns::OperationEnvironment *)> operate)
{
    upns::OperationEnvironmentImpl env( desc );
    env.setCheckout( this );
    upns::StatusCode status = operate( &env );
    OperationResult res(status, env.outputDescription());
    return res;
}

void upns::ZmqRequesterCheckout::syncHierarchy()
{
    std::unique_ptr<upns::RequestHierarchy> req(new upns::RequestHierarchy);
    req->set_checkout(m_checkoutName);
    m_node->send(std::move(req));
    upns::ReplyHierarchy *hierarchy = m_node->receive<upns::ReplyHierarchy>();
    assert(m_cache);
    //m_cache->

    delete hierarchy;
}

upns::StatusCode upns::ZmqRequesterCheckout::storeTree(const upns::Path &path, upnsSharedPointer<upns::Tree> tree)
{
    return UPNS_STATUS_ERR_NOT_YET_IMPLEMENTED;
}

upns::StatusCode upns::ZmqRequesterCheckout::storeEntity(const upns::Path &path, upnsSharedPointer<upns::Entity> entity)
{
    std::unique_ptr<upns::RequestStoreEntity> req(new upns::RequestStoreEntity);
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    req->set_type( entity->type() );
    req->set_offset(0ul);
    req->set_sendlength(0ul);
    req->set_entitylength(0ul);
    m_node->send(std::move(req));
    upns::upnsSharedPointer<upns::ReplyStoreEntity> rep(m_node->receive<upns::ReplyStoreEntity>());
    if(rep->status() == upns::ReplyStoreEntity::SUCCESS)
    {
        return UPNS_STATUS_OK;
    }
    else
    {
        return UPNS_STATUS_ERROR;
    }
}

upns::upnsSharedPointer<upns::AbstractEntityData> upns::ZmqRequesterCheckout::getEntityDataForReadWrite(const upns::Path &entity)
{
    upnsSharedPointer<Entity> e = getEntity(entity);
    if(!e)
    {
        log_error("Entity could not be queried for entitydata: " + entity);
        return upns::upnsSharedPointer<upns::AbstractEntityData>(nullptr);
    }
    upns::upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider(new ZmqEntitydataStreamProvider(m_checkoutName, entity, m_node));
    return EntityStreamManager::getEntityDataFromStreamImpl(e->type(), streamProvider);
}
