#include "upnszmqresponder_p.h"
#include "services.pb.h"
#include <functional>
#include "versioning/repository.h"

template < typename T, void (upns::ZmqResponderPrivate::*func)(T*) >
void upns::ZmqResponderPrivate::toDelegate(google::protobuf::Message* msg)
{
    (this->*func)(static_cast<T*>(msg));
}

upns::ZmqResponderPrivate::ZmqResponderPrivate(int portIncomingRequests, Repository *repo, std::__cxx11::string urlOutgoingRequests)
    :ZmqNode( true ),
      m_repo( repo ),
      m_urlOutgoing( urlOutgoingRequests ),
      m_portIncoming( portIncomingRequests )
{
    void (upns::ZmqResponderPrivate::*member)(google::protobuf::Message*);
    std::function<void(google::protobuf::Message*)> fn;

    member = &upns::ZmqResponderPrivate::toDelegate<RequestCheckout, &upns::ZmqResponderPrivate::handleRequestCheckout>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestCheckout>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestEntitydata, &upns::ZmqResponderPrivate::handleRequestEntitydata>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestEntitydata>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestHierarchy, &upns::ZmqResponderPrivate::handleRequestHierarchy>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestHierarchy>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestListCheckouts, &upns::ZmqResponderPrivate::handleRequestListCheckouts>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestListCheckouts>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestOperatorExecution, &upns::ZmqResponderPrivate::handleRequestOperatorExecution>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestOperatorExecution>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestStoreEntity, &upns::ZmqResponderPrivate::handleRequestStoreEntity>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestStoreEntity>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestEntity, &upns::ZmqResponderPrivate::handleRequestEntity>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestEntity>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestTree, &upns::ZmqResponderPrivate::handleRequestTree>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestTree>( fn );

    bind("tcp://*:" + std::to_string( m_portIncoming ) );
}

void upns::ZmqResponderPrivate::handleRequestCheckout(RequestCheckout *msg)
{
    std::unique_ptr<upns::ReplyCheckout> ptr(new upns::ReplyCheckout());
    upnsSharedPointer<Checkout> co = m_repo->getCheckout(msg->checkout());
    if(co == NULL)
    {
        co = m_repo->createCheckout(msg->commit(), msg->checkout());
        if( co == NULL )
        {
            ptr->set_status( upns::ReplyCheckout::ERROR );
        }
        else
        {
            ptr->set_status( upns::ReplyCheckout::SUCCESS );
        }
    }
    else
    {
        ptr->set_status( upns::ReplyCheckout::EXISTED );
    }
    send( std::move( ptr ) );
}

void upns::ZmqResponderPrivate::handleRequestEntitydata(RequestEntitydata *msg)
{
    std::unique_ptr<upns::ReplyEntitydata> ptr(new upns::ReplyEntitydata());
    upnsSharedPointer<Checkout> co = m_repo->getCheckout(msg->checkout());
    if(co == NULL)
    {
        ptr->set_status( upns::ReplyEntitydata::NOT_FOUND );
        ptr->set_length( 0 );
        send( std::move( ptr ) );
    }
    else
    {
        upnsSharedPointer<AbstractEntityData> ed = co->getEntitydataReadOnly( msg->entitypath() );
        ptr->set_status( upns::ReplyEntitydata::SUCCESS );
        ptr->set_length( ed->size() );
        // Send multipart message

        // protobuf frame
        send( std::move( ptr ), ZMQ_SNDMORE );

        // binary frames
        upnsIStream *strm = ed->startReadBytes();
        unsigned char buffer[4096];
        while (strm->read(reinterpret_cast<char*>(buffer), sizeof(buffer)))
        {
            send_raw_body( buffer, sizeof(buffer), ZMQ_SNDMORE );
        }
        send_raw_body( buffer, strm->gcount() );
        ed->endRead(strm);
    }
}

void upns::ZmqResponderPrivate::handleRequestHierarchy(RequestHierarchy *msg)
{
    // Note: At the moment this is the only place, where "/map/layer/entity" structure is hardcoded.
    std::unique_ptr<upns::ReplyHierarchy> rep(new upns::ReplyHierarchy());
    upnsSharedPointer<Checkout> co = m_repo->getCheckout(msg->checkout());
    if(co == NULL)
    {
        // TODO: Introduce Error-type
        //ptr->set_status( upns::ReplyHierarchy::CHECKOUT_NOT_FOUND );
        send( std::move( rep ) );
    }
    else
    {
        //ptr->set_status( upns::ReplyHierarchy::SUCCESS );


        upnsSharedPointer<Tree> root = co->getRoot();

        for(google::protobuf::Map< ::std::string, ::upns::ObjectReference >::const_iterator cmap( root->refs().cbegin() );
            cmap != root->refs().cend();
            ++cmap)
        {
            upns::ReplyHierarchyMap treeLevel0;
            upnsSharedPointer<Tree> map = co->getTree(cmap->first);
            for(google::protobuf::Map< ::std::string, ::upns::ObjectReference >::const_iterator clayer( map->refs().cbegin() );
                clayer != map->refs().cend();
                ++clayer)
            {
                upns::ReplyHierarchyLayer treeLevel1;
                //Path layerPath( cmap->first + "/" + clayer->first );
                upnsSharedPointer<Tree> layer = m_repo->getTree( clayer->second.id() );
                for(google::protobuf::Map< ::std::string, ::upns::ObjectReference >::const_iterator cent( layer->refs().cbegin() );
                    cent != layer->refs().cend();
                    ++cent)
                {
                    upnsSharedPointer<Entity> ent = m_repo->getEntity( cent->second.id() );
                    if(ent)
                    {
                        treeLevel1.mutable_entities()->insert(::google::protobuf::MapPair< ::std::string, ::upns::LayerType>( cent->first, ent->type()));
                    }
                    else
                    {
                        log_warn("Repository malformed. (/map/layer/entity)");
                    }
                }
                treeLevel0.mutable_layers()->insert(::google::protobuf::MapPair< ::std::string, ::upns::ReplyHierarchyLayer>( clayer->first, treeLevel1));
            }
            rep->mutable_maps()->insert(::google::protobuf::MapPair< ::std::string, ::upns::ReplyHierarchyMap>( cmap->first, treeLevel0));
        }
        send(std::move(rep));
//        upns::StatusCode s = co->depthFirstSearch([&](
//            upns::upnsSharedPointer<upns::Commit> obj, const upns::ObjectId& oid, const upns::Path &path)
//            {
//                //before commit
//                return true;
//            },
//            [&](upns::upnsSharedPointer<upns::Commit> obj, const upns::ObjectId& oid, const upns::Path &path)
//            {
//                //after commit
//                return true;
//            },
//            [&](upns::upnsSharedPointer<upns::Tree> obj, const upns::ObjectId& oid, const upns::Path &path)
//            {
//                //before tree
//                return true;
//            }, [&](upns::upnsSharedPointer<upns::Tree> obj, const upns::ObjectId& oid, const upns::Path &path)
//            {
//                //after tree
//                return true;
//            },
//            [&](upns::upnsSharedPointer<upns::Entity> obj, const upns::ObjectId& oid, const upns::Path &path)
//            {
//                //before entity
//                return true;
//            },
//            [&](upns::upnsSharedPointer<upns::Entity> obj, const upns::ObjectId& oid, const upns::Path &path)
//            {
//                //after entity
//                return true;
//            });
    }
}

void upns::ZmqResponderPrivate::handleRequestListCheckouts(RequestListCheckouts *msg)
{
    std::unique_ptr<upns::ReplyListCheckouts> rep(new upns::ReplyListCheckouts());
    upnsVec<upnsString> cos = m_repo->listCheckoutNames();
    for(upnsVec<upnsString>::const_iterator iter(cos.cbegin()) ; iter != cos.cend() ; ++iter)
    {
        rep->add_checkouts(*iter);
    }
    send( std::move( rep ) );
}

void upns::ZmqResponderPrivate::handleRequestOperatorExecution(RequestOperatorExecution *msg)
{
    std::unique_ptr<upns::ReplyOperatorExecution> rep(new upns::ReplyOperatorExecution());
    upnsSharedPointer<Checkout> co = m_repo->getCheckout(msg->checkout());
    upns::OperationResult result = co->doOperation(msg->param());
    rep->set_status_code(result.first);
    rep->set_error_msg("");
    send( std::move( rep ) );
}

void upns::ZmqResponderPrivate::handleRequestStoreEntity(RequestStoreEntity *msg)
{

}

void upns::ZmqResponderPrivate::handleRequestEntity(upns::RequestEntity *msg)
{
    std::unique_ptr<upns::ReplyEntity> rep(new upns::ReplyEntity());
    upnsSharedPointer<Checkout> co = m_repo->getCheckout(msg->checkout());
    upnsSharedPointer<Entity> e = co->getEntity(msg->path());
    if(e)
    {
        rep->set_status( upns::ReplyEntity::SUCCESS );
        rep->set_allocated_entity( new Entity(*e) );
    }
    else
    {
        rep->set_status( upns::ReplyEntity::NOT_FOUND );
    }
    send( std::move( rep ) );
}

void upns::ZmqResponderPrivate::handleRequestTree(upns::RequestTree *msg)
{
    std::unique_ptr<upns::ReplyTree> rep(new upns::ReplyTree());
    upnsSharedPointer<Checkout> co = m_repo->getCheckout(msg->checkout());
    upnsSharedPointer<Tree> t = co->getTree(msg->path());
    if(t)
    {
        rep->set_status( upns::ReplyTree::SUCCESS );
        rep->set_allocated_tree( new Tree(*t) );
    }
    else
    {
        rep->set_status( upns::ReplyTree::NOT_FOUND );
    }
    send( std::move( rep ) );
}
