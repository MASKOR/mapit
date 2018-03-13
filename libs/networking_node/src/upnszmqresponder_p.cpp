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

#define NOMINMAX
#include "upnszmqresponder_p.h"
#include <mapit/msgs/services.pb.h>
#include <functional>
#include <upns/versioning/repository.h>
#include <upns/operators/operationenvironment.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/errorcodes.h>

template < typename T, void (upns::ZmqResponderPrivate::*func)(T*) >
void upns::ZmqResponderPrivate::toDelegate(google::protobuf::Message* msg)
{
    (this->*func)(static_cast<T*>(msg));
}

upns::ZmqResponderPrivate::ZmqResponderPrivate(int portIncomingRequests, Repository *repo, std::string urlOutgoingRequests)
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

//    member = &upns::ZmqResponderPrivate::toDelegate<RequestStoreTree, &upns::ZmqResponderPrivate::handleRequestStoreTree>;
//    fn = std::bind(member, this, std::placeholders::_1);
//    add_receivable_message_type<RequestStoreTree>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestDeleteEntity, &upns::ZmqResponderPrivate::handleRequestDeleteEntity>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestDeleteEntity>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestDeleteTree, &upns::ZmqResponderPrivate::handleRequestDeleteTree>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestDeleteTree>( fn );

    //TODO: allow ssh://?
    bind("tcp://*:" + std::to_string( m_portIncoming ) );
}

void upns::ZmqResponderPrivate::handleRequestCheckout(RequestCheckout *msg)
{
    std::unique_ptr<ReplyCheckout> ptr(new ReplyCheckout());
    std::shared_ptr<Checkout> co = m_repo->getCheckout(msg->checkout());
    if(co == NULL)
    {
        if(msg->createifnotexists())
        {
            //TODO: handle multiple commits for merge
            CommitId commit = msg->commit_size() == 0 ? "" : msg->commit(0);
            co = m_repo->createCheckout(commit, msg->checkout());
            if( co == NULL )
            {
                log_info("Could not get checkout \"" + msg->checkout() + "\"");
                ptr->set_status( ReplyCheckout::ERROR );
            }
            else
            {
                *ptr->mutable_commit() = msg->commit();
                ptr->set_status( ReplyCheckout::SUCCESS );
            }
        }
        else
        {
            ptr->set_status( ReplyCheckout::SUCCESS );
        }
    }
    else
    {
        std::vector<CommitId> ids(co->getParentCommitIds());
        for(std::vector<CommitId>::const_iterator citer(ids.cbegin()); citer != ids.cend() ; ++citer)
        {
            ptr->add_commit( *citer );
        }
        ptr->set_status( ReplyCheckout::EXISTED );
    }
    send( std::move( ptr ) );
}

void upns::ZmqResponderPrivate::handleRequestEntitydata(RequestEntitydata *msg)
{
    try {
        std::unique_ptr<ReplyEntitydata> ptr(new ReplyEntitydata());

        // Validate input
        std::shared_ptr<Checkout> co = m_repo->getCheckout(msg->checkout());
        if(co == NULL)
        {
            ptr->set_status( ReplyEntitydata::NOT_FOUND );
            ptr->set_receivedlength( 0 );
            ptr->set_entitylength( 0 );
            send( std::move( ptr ) );
            return;
        }
        std::shared_ptr<AbstractEntitydata> ed = co->getEntitydataReadOnly( msg->entitypath() );

        ReplyEntitydata::Status status;
        if(ed == nullptr )
        {
            log_info("Could not find requested entitydata \"" + msg->entitypath() + "\"");
            status = ReplyEntitydata::NOT_FOUND;
        }
        else
        {
            if(msg->offset() > ed->size())
            {
                log_warn("Server got request with offset exceeding entitydata size.");
                status = ReplyEntitydata::EXCEEDED_BOUNDARY;
            }
    //        else if(msg->offset() + msg->maxlength() > ed->size())
    //        {
                  // This is absolutly okay
    //            log_warn("Server got request trying to read across entitydata size boundary.");
    //            status = upns::ReplyEntitydata::EXCEEDED_BOUNDARY;
    //        }
            else
            {
                status = ReplyEntitydata::SUCCESS;
            }
        }
        if( status != ReplyEntitydata::SUCCESS )
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

        try {
            // protobuf frame
            send( std::move( ptr ), ZMQ_SNDMORE );
        }
        catch(zmq::error_t err )
        {
            log_error("Zmq Responder could not reply with entitydata (handleRequestEntitydata): " + err.what());
            unsigned char buffer[0];
            send_raw_body( buffer, 0 );
            return;
        }
        // binary frames
        upnsIStream *strm = ed->startReadBytes( offset, len);
        try
        {
            unsigned char buffer[4096];
            while (strm->read(reinterpret_cast<char*>(buffer), sizeof(buffer)))
            {
                send_raw_body( buffer, sizeof(buffer), ZMQ_SNDMORE );
            }
            int remaining = strm->gcount();
            send_raw_body( buffer, remaining );
        }
        catch(zmq::error_t err )
        {
            log_error("Zmq Responder could not reply with entitydata (handleRequestEntitydata (zmq error)): " + err.what());
            ed->endRead(strm);
            unsigned char buffer[0];
            send_raw_body( buffer, 0 );
            return;
        }
        catch(...)
        {
            log_error("Zmq Responder could not reply with entitydata (handleRequestEntitydata)");
            ed->endRead(strm);
            return;
        }
        ed->endRead(strm);
    }
    catch(...)
    {
        log_error("Zmq Responder could not reply with entitydata (handleRequestEntitydata)");
        return;
    }
}

void upns::ZmqResponderPrivate::handleRequestHierarchy(RequestHierarchy *msg)
{
    //TODO: Remove from service definition
    //Note that everything this message does can be done using internal messages "getTree" and "getEntity".
    std::unique_ptr<ReplyHierarchy> rep(new ReplyHierarchy());
    send(std::move(rep));
}

void upns::ZmqResponderPrivate::handleRequestHierarchyPlain(RequestHierarchyPlain *msg)
{
    std::unique_ptr<ReplyHierarchyPlain> rep(new ReplyHierarchyPlain());
    std::shared_ptr<Checkout> co = m_repo->getCheckout(msg->checkout());
    if(co == NULL)
    {
        // TODO: Introduce Error-type
        rep->set_status( ReplyHierarchyPlain::CHECKOUT_NOT_FOUND );
        send( std::move( rep ) );
    }
    else
    {
        StatusCode s = co->depthFirstSearch(
            [&](std::shared_ptr<Tree> obj, const ObjectReference &ref, const Path &path){return true;},
            [&](std::shared_ptr<Tree> obj, const ObjectReference &ref, const Path &path) {return true;},
            [&](std::shared_ptr<Entity> obj, const ObjectReference &ref, const Path &path)
            {
                rep->add_entities(path);
                return true;
            },
            [&](std::shared_ptr<Entity> obj, const ObjectReference &ref, const Path &path){return true;});
        if(!upnsIsOk(s))
        {
            log_info("error while listing entities (dfs)");
        }
    }
}

void upns::ZmqResponderPrivate::handleRequestListCheckouts(RequestListCheckouts *msg)
{
    std::unique_ptr<ReplyListCheckouts> rep(new ReplyListCheckouts());
    std::vector<std::string> cos = m_repo->listCheckoutNames();
    for(std::vector<std::string>::const_iterator iter(cos.cbegin()) ; iter != cos.cend() ; ++iter)
    {
        rep->add_checkouts(*iter);
    }
    send( std::move( rep ) );
}

void upns::ZmqResponderPrivate::handleRequestOperatorExecution(RequestOperatorExecution *msg)
{
    std::unique_ptr<ReplyOperatorExecution> rep(new ReplyOperatorExecution());
    std::shared_ptr<Checkout> co = m_repo->getCheckout(msg->checkout());
    if(co == NULL)
    {
        rep->set_status_code( UPNS_STATUS_ERR_DB_NOT_FOUND );
        rep->set_error_msg( "Checkout does not exist" );
        send( std::move( rep ) );
        return;
    }
    upns::OperationResult result = co->doOperation(msg->param());
    rep->set_status_code(result.first);
    rep->set_error_msg(""); // TODO: This is the success, errormessage. There are no more errormessages yet.
    send( std::move( rep ) );
}

void upns::ZmqResponderPrivate::handleRequestStoreEntity(RequestStoreEntity *msg)
{
    // This method handles two cases: "storeEntity" and "writeEntityData"
    std::unique_ptr<ReplyStoreEntity> rep(new ReplyStoreEntity());
    // Input validation
    // Entity size and offset, don't write out-of-bounds
    if(msg->offset() > msg->sendlength())
    {
        discard_more();
        log_info("Write offset for entity \"" + msg->path() + "\" was specified wrong.");
        rep->set_status(ReplyStoreEntity::ERROR);
        send( std::move( rep ) );
        return;
    }
    if(msg->sendlength() > msg->entitylength())
    {
        discard_more();
        log_info("Entity \"" + msg->path() + "\" is too small for received data or parameters sendlength and entitylength are wrong.");
        rep->set_status(ReplyStoreEntity::ERROR);
        send( std::move( rep ) );
        return;
    }
    std::shared_ptr<Checkout> co = m_repo->getCheckout(msg->checkout());
    if(co == NULL)
    {
        discard_more();
        log_info("Checkout was not available");
        rep->set_status(ReplyStoreEntity::ERROR);
        send( std::move( rep ) );
        return;
    }
    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("StoreEntity");
    desc.set_params("{source:\"network\"}");

    upns::OperationResult res = co->doUntraceableOperation(desc, [&msg, this](upns::OperationEnvironment *env){
        upns::CheckoutRaw* coraw = env->getCheckout();

        // if offset is not 0, we assume the entity already exists.
        if(msg->offset() == 0)
        {
            // Receive Entity out of the message and compare to existing entity
            std::shared_ptr<Entity> entity(coraw->getEntity(msg->path()));
            if(nullptr == entity)
            {
                // There was no entity yet
                entity = std::shared_ptr<Entity>(new Entity);
            }
            if( !entity->type().empty()
               && !msg->type().empty()
               && entity->type().compare(msg->type()) != 0)
            {
                // Changing layertype
                log_info("Entity " + msg->path() + " changed its layertype");
                entity->set_type(msg->type());
            }
            if(entity->type().empty() && !msg->type().empty())
            {
                // No layertype was set before
                entity->set_type(msg->type());
            }
            if(entity->type().empty() && msg->type().empty())
            {
                // No layertype at all (error)
                log_info("Tried to write entitydata, but there was no type information in this or previous communication.");
                return UPNS_STATUS_INVALID_ARGUMENT;
            }
            else
            {
                // Other cases: There was a layertype and none is set now (just leave it as is).
                upns::StatusCode status = coraw->storeEntity(msg->path(), entity);
                if(!upnsIsOk(status))
                {
                    log_info("Could not store entity: \"" + msg->path() + "\"");
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
            log_info("Server expected raw data (" + std::to_string(msg->sendlength()) + " bytes for entity \"" + msg->path() + "\"), but nothing was received.");
            return UPNS_STATUS_INVALID_ARGUMENT;
        }
        // write entitydata
        std::shared_ptr<AbstractEntitydata> ed = coraw->getEntitydataForReadWrite(msg->path());
        if(ed == nullptr)
        {
            log_info("Entitydata of type could not be created.");
            return UPNS_STATUS_ERR_MODULE_OPERATOR_NOT_FOUND;
        }
        upns::upnsOStream *stream = ed->startWriteBytes(msg->offset(), msg->sendlength());
        size_t offset = 0;
        while(has_more())
        {
            std::unique_ptr<zmq::message_t> buf( receive_raw_body() );
            if(buf->size() + offset > msg->sendlength())
            {
                log_info("Tried to store entitydata with sendlength smaller than received datasize.");
                ed->endWrite(stream);
                return UPNS_STATUS_INVALID_ARGUMENT;
            }
            stream->write(static_cast<char*>(buf->data()), buf->size());
            offset += buf->size();
        }
        if(offset != msg->sendlength())
        {
            log_info("Received entitydata of wrong length: should: " + std::to_string(msg->sendlength()) + ", is: " + std::to_string(offset) + "." );
            ed->endWrite(stream);
            return UPNS_STATUS_INVALID_DATA;
        }
        ed->endWrite(stream);
        return UPNS_STATUS_OK;
    });
    if(upnsIsOk(res.first))
    {
        assert(!has_more());
        rep->set_status(ReplyStoreEntity::SUCCESS);
    }
    else
    {
        log_info("Could not store entity \"" + msg->path() + "\" due to an error during inline operator. (" + std::to_string(res.first) + ")");
        rep->set_status(ReplyStoreEntity::ERROR);
    }
    send( std::move( rep ) );
}

//void upns::ZmqResponderPrivate::handleRequestStoreTree(RequestStoreTree *msg)
//{
//    std::unique_ptr<ReplyStoreTree> rep(new ReplyStoreTree());

//    // Validate input
//    std::shared_ptr<Checkout> checkout = m_repo->getCheckout(msg->checkout());
//    if(checkout == NULL)
//    {
//        rep->set_status( ReplyStoreTree::CHECKOUT_NOT_FOUND );
//        send( std::move( rep ) );
//        return;
//    }
//    MessageType type = checkout->typeOfObject(msg->path());
//    if(type != MessageEmpty)
//    {
//        rep->set_status( ReplyStoreTree::EXISTED );
//        send( std::move( rep ) );
//        return;
//    }
//    OperationDescription desc;
//    desc.mutable_operator_()->set_operatorname("StoreTree");
//    desc.set_params("{source:\"network\"}");
//    upns::OperationResult res = checkout->doUntraceableOperation(desc, [&msg, this](upns::OperationEnvironment *env){
//        upns::CheckoutRaw* coraw = env->getCheckout();
//        std::shared_ptr<Tree> tree;
//        StatusCode s = coraw->storeTree(msg->path(), tree);
//        return s;
//    });
//    if(upnsIsOk(res.first))
//    {
//        assert(!has_more());
//        rep->set_status(ReplyStoreTree::SUCCESS);
//    }
//    else
//    {
//        log_info("Could not store tree \"" + msg->path() + "\" due to an error during inline operator. (" + std::to_string(res.first) + ")");
//        rep->set_status(ReplyStoreTree::ERROR);
//    }
//    send( std::move( rep ) );
//}

void
upns::ZmqResponderPrivate::handleRequestDeleteEntity(RequestDeleteEntity* msg)
{
    std::unique_ptr<ReplyDeleteEntity> rep = std::make_unique<ReplyDeleteEntity>();
    std::shared_ptr<Checkout> co = m_repo->getCheckout(msg->checkout());
    if(co == NULL)
    {
        discard_more();
        log_info("Checkout was not available");
        rep->set_status(ReplyDeleteEntity::ERROR);
        send( std::move( rep ) );
        return;
    }
    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("DeleteEntity");
    desc.set_params("{source:\"network\"}");

    upns::OperationResult res = co->doUntraceableOperation(desc, [&msg, this](upns::OperationEnvironment *env){
        upns::CheckoutRaw* coraw = env->getCheckout();

        return coraw->deleteEntity(msg->path());
    });
    if(upnsIsOk(res.first))
    {
        rep->set_status(ReplyDeleteEntity::SUCCESS);
    }
    else
    {
        log_info("Could not delete entity \"" + msg->path() + "\" due to an error during inline operator. (" + std::to_string(res.first) + ")");
        rep->set_status(ReplyDeleteEntity::ERROR);
    }
    send( std::move( rep ) );
}


void
upns::ZmqResponderPrivate::handleRequestDeleteTree(RequestDeleteTree* msg)
{
    std::unique_ptr<ReplyDeleteTree> rep = std::make_unique<ReplyDeleteTree>();
    std::shared_ptr<Checkout> co = m_repo->getCheckout(msg->checkout());
    if(co == NULL)
    {
        discard_more();
        log_info("Checkout was not available");
        rep->set_status(ReplyDeleteTree::ERROR);
        send( std::move( rep ) );
        return;
    }
    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("DeleteTree");
    desc.set_params("{source:\"network\"}");

    upns::OperationResult res = co->doUntraceableOperation(desc, [&msg, this](upns::OperationEnvironment *env){
        upns::CheckoutRaw* coraw = env->getCheckout();

        return coraw->deleteTree(msg->path());
    });
    if(upnsIsOk(res.first))
    {
        rep->set_status(ReplyDeleteTree::SUCCESS);
    }
    else
    {
        log_info("Could not delete tree \"" + msg->path() + "\" due to an error during inline operator. (" + std::to_string(res.first) + ")");
        rep->set_status(ReplyDeleteTree::ERROR);
    }
    send( std::move( rep ) );
}

void upns::ZmqResponderPrivate::handleRequestGenericEntry(RequestGenericEntry *msg)
{
    Replier<ReplyGenericEntry> rep(new ReplyGenericEntry(), this);
    //std::unique_ptr<upns::ReplyGenericEntry> rep(new upns::ReplyGenericEntry());
    std::shared_ptr<Checkout> co = m_repo->getCheckout(msg->checkout());
    if(co == nullptr)
    {
        log_info("Checkout for requestGenericEntry does not exist");
        rep.reply()->set_status( ReplyGenericEntry::NOT_FOUND );
        rep.send();
        return;
    }
    MessageType type = co->typeOfObject(msg->path());
    if(type == MessageTree)
    {
        std::shared_ptr<Tree> tree = co->getTree(msg->path());
        if(tree)
        {
            rep.reply()->set_status( ReplyGenericEntry::SUCCESS );
            rep.reply()->mutable_entry()->set_allocated_tree( new Tree(*tree) );
        }
        else
        {
            log_info("Tree \"" + msg->path() + "\" was requested but not found.");
            rep.reply()->set_status( ReplyGenericEntry::NOT_FOUND );
        }
    }
    else if(type == MessageEntity)
    {
        std::shared_ptr<Entity> entity = co->getEntity(msg->path());
        if(entity)
        {
            rep.reply()->set_status( ReplyGenericEntry::SUCCESS );
            rep.reply()->mutable_entry()->set_allocated_entity( new Entity(*entity) );
        }
        else
        {
            log_info("Entity \"" + msg->path() + "\" was requested but not found.");
            rep.reply()->set_status( ReplyGenericEntry::NOT_FOUND );
        }
    }
    rep.send();
}

//void upns::ZmqResponderPrivate::handleRequestTree(upns::RequestTree *msg)
//{
//    std::unique_ptr<upns::ReplyTree> rep(new upns::ReplyTree());
//    std::shared_ptr<Checkout> co = m_repo->getCheckout(msg->checkout());
//    std::shared_ptr<Tree> t = co->getTree(msg->path());
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
