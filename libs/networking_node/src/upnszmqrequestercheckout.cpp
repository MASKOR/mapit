#include "upnszmqrequestercheckout.h"
#include "serialization/entitystreammanager.h" //TODO: put in yet another independent project/cmake target. No dependecy to mapmanager required.
#include "serialization/zmqentitydatastreamprovider.h"

upns::ZmqRequesterCheckout::ZmqRequesterCheckout(upnsString name, ZmqNode *node, Checkout *cache)
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
    //TODO: introduce internal datatype
    return upns::upnsSharedPointer<upns::Tree>();
}

upns::upnsSharedPointer<upns::Entity> upns::ZmqRequesterCheckout::getEntity(const upns::Path &path)
{
    //TODO: introduce internal datatype
    return upns::upnsSharedPointer<upns::Entity>();
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
    std::unique_ptr<upns::RequestEntitydata> req(new upns::RequestEntitydata);
    req->set_checkout(m_checkoutName);
    req->set_entitypath(entityId);
    m_node->send(std::move(req));
    upns::upnsSharedPointer<upns::ReplyEntitydata> rep(m_node->receive<upns::ReplyEntitydata>());
    if(rep->status() == upns::ReplyEntitydata::SUCCESS)
    {
        char buf[1024];
        int64_t more;
        do {
            m_node->receive_raw_body(buf, sizeof(buf));
        } while (m_node->has_more());
        upns::upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider(new ZmqEntitydataStreamProvider(m_checkoutName, entityId, m_node));
        return EntityStreamManager::getEntityDataFromStreamImpl(e->type(), streamProvider);
    }
    else
    {
        return upns::upnsSharedPointer<upns::AbstractEntityData>(nullptr);
    }
}

upns::upnsSharedPointer<upns::AbstractEntityData> upns::ZmqRequesterCheckout::getEntitydataReadOnlyConflict(const upns::ObjectId &entityId)
{

}

upns::StatusCode upns::ZmqRequesterCheckout::depthFirstSearch(std::function<bool (upns::upnsSharedPointer<upns::Commit>, const upns::ObjectId &, const upns::Path &)> beforeCommit, std::function<bool (upns::upnsSharedPointer<upns::Commit>, const upns::ObjectId &, const upns::Path &)> afterCommit, std::function<bool (upns::upnsSharedPointer<upns::Tree>, const upns::ObjectId &, const upns::Path &)> beforeTree, std::function<bool (upns::upnsSharedPointer<upns::Tree>, const upns::ObjectId &, const upns::Path &)> afterTree, std::function<bool (upns::upnsSharedPointer<upns::Entity>, const upns::ObjectId &, const upns::Path &)> beforeEntity, std::function<bool (upns::upnsSharedPointer<upns::Entity>, const upns::ObjectId &, const upns::Path &)> afterEntity)
{

}

upns::OperationResult upns::ZmqRequesterCheckout::doOperation(const upns::OperationDescription &desc)
{

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
