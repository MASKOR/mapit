#include "upnszmqresponder_p.h"
#include "services.pb.h"
#include <functional>
#include "versioning/repository.h"
#include "modules/operationenvironment.h"
#include "modules/versioning/checkoutraw.h"
#include "upns_errorcodes.h"

template < typename T, void (upns::ZmqResponderPrivate::*func)(T*) >
void upns::ZmqResponderPrivate::toDelegate(google::protobuf::Message* msg)
{
    (this->*func)(static_cast<T*>(msg));
}

upns::ZmqResponderPrivate::ZmqResponderPrivate(int portIncomingRequests, Repository *repo, std::__cxx11::string urlOutgoingRequests)
    :ZmqProtobufNode( true ),
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

    member = &upns::ZmqResponderPrivate::toDelegate<RequestHierarchyPlain, &upns::ZmqResponderPrivate::handleRequestHierarchyPlain>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestHierarchyPlain>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestListCheckouts, &upns::ZmqResponderPrivate::handleRequestListCheckouts>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestListCheckouts>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestOperatorExecution, &upns::ZmqResponderPrivate::handleRequestOperatorExecution>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestOperatorExecution>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestGenericEntry, &upns::ZmqResponderPrivate::handleRequestGenericEntry>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestGenericEntry>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestStoreEntity, &upns::ZmqResponderPrivate::handleRequestStoreEntity>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestStoreEntity>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestStoreTree, &upns::ZmqResponderPrivate::handleRequestStoreTree>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestStoreTree>( fn );

    bind("tcp://*:" + std::to_string( m_portIncoming ) );
}

void upns::ZmqResponderPrivate::handleRequestCheckout(RequestCheckout *msg)
{
    std::unique_ptr<upns::ReplyCheckout> ptr(new upns::ReplyCheckout());
    upnsSharedPointer<Checkout> co = m_repo->getCheckout(msg->checkout());
    if(co == NULL)
    {
        if(msg->createifnotexists())
        {
            //TODO: handle multiple commits for merge
            CommitId commit = msg->commit_size() == 0 ? "" : msg->commit(0);
            co = m_repo->createCheckout(commit, msg->checkout());
            if( co == NULL )
            {
                log_error("Could not get checkout \"" + msg->checkout() + "\"");
                ptr->set_status( upns::ReplyCheckout::ERROR );
            }
            else
            {
                *ptr->mutable_commit() = msg->commit();
                ptr->set_status( upns::ReplyCheckout::SUCCESS );
            }
        }
        else
        {
            ptr->set_status( upns::ReplyCheckout::SUCCESS );
        }
    }
    else
    {
        upnsVec<CommitId> ids(co->getParentCommitIds());
        for(upnsVec<CommitId>::const_iterator citer(ids.cbegin()); citer != ids.cend() ; ++citer)
        {
            ptr->add_commit( *citer );
        }
        ptr->set_status( upns::ReplyCheckout::EXISTED );
    }
    send( std::move( ptr ) );
}

void upns::ZmqResponderPrivate::handleRequestEntitydata(RequestEntitydata *msg)
{
    std::unique_ptr<upns::ReplyEntitydata> ptr(new upns::ReplyEntitydata());

    // Validate input
    upnsSharedPointer<Checkout> co = m_repo->getCheckout(msg->checkout());
    if(co == NULL)
    {
        ptr->set_status( upns::ReplyEntitydata::NOT_FOUND );
        ptr->set_receivedlength( 0 );
        ptr->set_entitylength( 0 );
        send( std::move( ptr ) );
        return;
    }
    upnsSharedPointer<AbstractEntitydata> ed = co->getEntitydataReadOnly( msg->entitypath() );

    upns::ReplyEntitydata::Status status;
    if(ed == nullptr )
    {
        log_info("Could not find requested entitydata \"" + msg->entitypath() + "\"");
        status = upns::ReplyEntitydata::NOT_FOUND;
    }
    else
    {
        if(msg->offset() > ed->size())
        {
            log_warn("Server got request with offset exceeding entitydata size.");
            status = upns::ReplyEntitydata::EXCEEDED_BOUNDARY;
        }
//        else if(msg->offset() + msg->maxlength() > ed->size())
//        {
              // This is absolutly okay
//            log_warn("Server got request trying to read across entitydata size boundary.");
//            status = upns::ReplyEntitydata::EXCEEDED_BOUNDARY;
//        }
        else
        {
            status = upns::ReplyEntitydata::SUCCESS;
        }
    }
    if( status != upns::ReplyEntitydata::SUCCESS )
    {
        // Invalid arguments
        ptr->set_status( status );
        ptr->set_receivedlength( 0 );
        ptr->set_entitylength( ed == nullptr ? 0 : ed->size() );

        // protobuf frame
        send( std::move( ptr ) );
        return;
    }

    upnsuint64 offset = msg->offset(), len = msg->maxlength();
    if(len == 0)
    {
        len = ed->size()-offset;
    }
    else
    {
        len = std::min(len, ed->size()-offset);
    }

    ptr->set_status( status );
    ptr->set_receivedlength( len );
    ptr->set_entitylength( ed->size() );

    // Send multipart message

    // protobuf frame
    send( std::move( ptr ), ZMQ_SNDMORE );

    // binary frames
    upnsIStream *strm = ed->startReadBytes( offset, len);
    unsigned char buffer[4096];
    while (strm->read(reinterpret_cast<char*>(buffer), sizeof(buffer)))
    {
        send_raw_body( buffer, sizeof(buffer), ZMQ_SNDMORE );
    }
    int remaining = strm->gcount();
    send_raw_body( buffer, remaining );
    ed->endRead(strm);
}

void upns::ZmqResponderPrivate::handleRequestHierarchy(RequestHierarchy *msg)
{
    // TODO: Can only use paths/transient data at the moment
    // This is TODO: Write test and make it pass. Note that everything this message does can be done using internal messages "getTree" and "getEntity".
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
                upnsSharedPointer<Tree> layer = co->getTree( clayer->second.path() );
                assert(layer); //TODO? Just reimplement request hierechy and interface
                for(google::protobuf::Map< ::std::string, ::upns::ObjectReference >::const_iterator cent( layer->refs().cbegin() );
                    cent != layer->refs().cend();
                    ++cent)
                {
                    upnsSharedPointer<Entity> ent = co->getEntity( cent->second.path() );
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

void upns::ZmqResponderPrivate::handleRequestHierarchyPlain(RequestHierarchyPlain *msg)
{
    std::unique_ptr<upns::ReplyHierarchyPlain> rep(new upns::ReplyHierarchyPlain());
    upnsSharedPointer<Checkout> co = m_repo->getCheckout(msg->checkout());
    if(co == NULL)
    {
        // TODO: Introduce Error-type
        rep->set_status( upns::ReplyHierarchyPlain::CHECKOUT_NOT_FOUND );
        send( std::move( rep ) );
    }
    else
    {
        StatusCode s = co->depthFirstSearch(
            [&](upnsSharedPointer<Commit> obj, const ObjectReference &ref, const Path &path){return true;},
            [&](upnsSharedPointer<Commit> obj, const ObjectReference &ref, const Path &path) {return true;},
            [&](upnsSharedPointer<Tree> obj, const ObjectReference &ref, const Path &path){return true;},
            [&](upnsSharedPointer<Tree> obj, const ObjectReference &ref, const Path &path) {return true;},
            [&](upnsSharedPointer<Entity> obj, const ObjectReference &ref, const Path &path)
            {
                rep->add_entities(path);
                return true;
            },
            [&](upnsSharedPointer<Entity> obj, const ObjectReference &ref, const Path &path){return true;});
        if(!upnsIsOk(s))
        {
            log_error("error while listing entities (dfs)");
        }
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
    rep->set_error_msg(""); // TODO: This is the success, errormessage. There are no more errormessages yet.
    send( std::move( rep ) );
}

void upns::ZmqResponderPrivate::handleRequestStoreEntity(RequestStoreEntity *msg)
{
    // This method handles two cases: "storeEntity" and "writeEntityData"
    std::unique_ptr<upns::ReplyStoreEntity> rep(new upns::ReplyStoreEntity());
    // Input validation
    // Entity size and offset, don't write out-of-bounds
    if(msg->offset() > msg->sendlength())
    {
        discard_more();
        log_error("Write offset for entity \"" + msg->path() + "\" was specified wrong.");
        rep->set_status(upns::ReplyStoreEntity::ERROR);
        send( std::move( rep ) );
        return;
    }
    if(msg->sendlength() > msg->entitylength())
    {
        discard_more();
        log_error("Entity \"" + msg->path() + "\" is too small for received data or parameters sendlength and entitylength are wrong.");
        rep->set_status(upns::ReplyStoreEntity::ERROR);
        send( std::move( rep ) );
        return;
    }
    upnsSharedPointer<Checkout> co = m_repo->getCheckout(msg->checkout());
    OperationDescription desc;
    desc.set_operatorname("StoreEntity");
    desc.set_params("{source:\"network\"}");

    upns::OperationResult res = co->doUntraceableOperation(desc, [&msg, this](upns::OperationEnvironment *env){
        upns::CheckoutRaw* coraw = env->getCheckout();

        // if offset is not 0, we assume the entity already exists.
        if(msg->offset() == 0)
        {
            // Receive Entity out of the message and compare to existing entity
            upns::upnsSharedPointer<upns::Entity> entity(coraw->getEntity(msg->path()));
            if(nullptr == entity)
            {
                // There was no entity yet
                entity = upns::upnsSharedPointer<upns::Entity>(new upns::Entity);
            }
            if( entity->type()   != 0
               && msg->type()    != 0
               && entity->type() != msg->type())
            {
                // Changing layertype
                log_info("Entity " + msg->path() + " changed its layertype");
                entity->set_type(msg->type());
            }
            if(entity->type() == 0 && msg->type() != 0)
            {
                // No layertype was set before
                entity->set_type(msg->type());
            }
            if(entity->type() == 0 && msg->type() == 0)
            {
                // No layertype at all (error)
                log_error("Tried to write entitydata, but there was no type information in this or previous communication.");
                return UPNS_STATUS_INVALID_ARGUMENT;
            }
            else
            {
                // Other cases: There was a layertype and none is set now (just leave it as is).
                upns::StatusCode status = coraw->storeEntity(msg->path(), entity);
                if(!upnsIsOk(status))
                {
                    log_error("Could not store entity: \"" + msg->path() + "\"");
                    return status;
                }
            }
        }
        if(msg->sendlength() == 0)
        {
            return UPNS_STATUS_OK;
        }
        if(!has_more())
        {
            log_error("Server expected raw data (" + std::to_string(msg->sendlength()) + " bytes for entity \"" + msg->path() + "\"), but nothing was received.");
            return UPNS_STATUS_INVALID_ARGUMENT;
        }
        // write entitydata
        upnsSharedPointer<AbstractEntitydata> ed = coraw->getEntitydataForReadWrite(msg->path());
        upns::upnsOStream *stream = ed->startWriteBytes(msg->offset(), msg->sendlength());
        size_t offset = 0;
        while(has_more())
        {
            std::unique_ptr<zmq::message_t> buf( receive_raw_body() );
            if(buf->size() + offset > msg->sendlength())
            {
                log_error("Tried to store entitydata with sendlength smaller than received datasize.");
                ed->endWrite(stream);
                return UPNS_STATUS_INVALID_ARGUMENT;
            }
            stream->write(static_cast<char*>(buf->data()), buf->size());
            offset += buf->size();
        }
        if(offset != msg->sendlength())
        {
            log_error("Received entitydata of wrong length: should: " + std::to_string(msg->sendlength()) + ", is: " + std::to_string(offset) + "." );
            ed->endWrite(stream);
            return UPNS_STATUS_INVALID_DATA;
        }
        ed->endWrite(stream);
        return UPNS_STATUS_OK;
    });
    if(upnsIsOk(res.first))
    {
        assert(!has_more());
        rep->set_status(upns::ReplyStoreEntity::SUCCESS);
    }
    else
    {
        log_error("Could not store entity \"" + msg->path() + "\" due to an error during inline operator. (" + std::to_string(res.first) + ")");
        rep->set_status(upns::ReplyStoreEntity::ERROR);
    }
    send( std::move( rep ) );
}

void upns::ZmqResponderPrivate::handleRequestStoreTree(upns::RequestStoreTree *msg)
{
    std::unique_ptr<upns::ReplyStoreTree> rep(new upns::ReplyStoreTree());

    // Validate input
    upnsSharedPointer<Checkout> checkout = m_repo->getCheckout(msg->checkout());
    if(checkout == NULL)
    {
        rep->set_status( upns::ReplyStoreTree::CHECKOUT_NOT_FOUND );
        send( std::move( rep ) );
        return;
    }
    MessageType type = checkout->typeOfObject(msg->path());
    if(type != MessageEmpty)
    {
        rep->set_status( upns::ReplyStoreTree::EXISTED );
        send( std::move( rep ) );
        return;
    }
    OperationDescription desc;
    desc.set_operatorname("StoreTree");
    desc.set_params("{source:\"network\"}");
    upns::OperationResult res = checkout->doUntraceableOperation(desc, [&msg, this](upns::OperationEnvironment *env){
        upns::CheckoutRaw* coraw = env->getCheckout();
        upnsSharedPointer<Tree> tree;
        StatusCode s = coraw->storeTree(msg->path(), tree);
        return s;
    });
    if(upnsIsOk(res.first))
    {
        assert(!has_more());
        rep->set_status(upns::ReplyStoreTree::SUCCESS);
    }
    else
    {
        log_error("Could not store tree \"" + msg->path() + "\" due to an error during inline operator. (" + std::to_string(res.first) + ")");
        rep->set_status(upns::ReplyStoreTree::ERROR);
    }
    send( std::move( rep ) );
}

void upns::ZmqResponderPrivate::handleRequestGenericEntry(upns::RequestGenericEntry *msg)
{
    Replier<upns::ReplyGenericEntry> rep(new upns::ReplyGenericEntry(), this);
    //std::unique_ptr<upns::ReplyGenericEntry> rep(new upns::ReplyGenericEntry());
    upnsSharedPointer<Checkout> co = m_repo->getCheckout(msg->checkout());
    MessageType type = co->typeOfObject(msg->path());
    if(type == MessageTree)
    {
        upnsSharedPointer<Tree> tree = co->getTree(msg->path());
        if(tree)
        {
            rep.reply()->set_status( upns::ReplyGenericEntry::SUCCESS );
            rep.reply()->mutable_entry()->set_allocated_tree( new Tree(*tree) );
        }
        else
        {
            log_info("Tree \"" + msg->path() + "\" was requested but not found.");
            rep.reply()->set_status( upns::ReplyGenericEntry::NOT_FOUND );
        }
    }
    else if(type == MessageEntity)
    {
        upnsSharedPointer<Entity> entity = co->getEntity(msg->path());
        if(entity)
        {
            rep.reply()->set_status( upns::ReplyGenericEntry::SUCCESS );
            rep.reply()->mutable_entry()->set_allocated_entity( new Entity(*entity) );
        }
        else
        {
            log_info("Entity \"" + msg->path() + "\" was requested but not found.");
            rep.reply()->set_status( upns::ReplyGenericEntry::NOT_FOUND );
        }
    }
    rep.send();
}

//void upns::ZmqResponderPrivate::handleRequestTree(upns::RequestTree *msg)
//{
//    std::unique_ptr<upns::ReplyTree> rep(new upns::ReplyTree());
//    upnsSharedPointer<Checkout> co = m_repo->getCheckout(msg->checkout());
//    upnsSharedPointer<Tree> t = co->getTree(msg->path());
//    if(t)
//    {
//        rep->set_status( upns::ReplyTree::SUCCESS );
//        rep->set_allocated_tree( new Tree(*t) );
//    }
//    else
//    {
//        rep->set_status( upns::ReplyTree::NOT_FOUND );
//    }
//    send( std::move( rep ) );
//}
