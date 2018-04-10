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
#include "zmqresponder_p.h"
#include <mapit/msgs/services.pb.h>
#include <functional>
#include <mapit/versioning/repository.h>
#include <mapit/operators/operationenvironment.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/errorcodes.h>

template < typename T, void (mapit::ZmqResponderPrivate::*func)(T*) >
void mapit::ZmqResponderPrivate::toDelegate(google::protobuf::Message* msg)
{
    (this->*func)(static_cast<T*>(msg));
}

mapit::ZmqResponderPrivate::ZmqResponderPrivate(int portIncomingRequests, Repository *repo, std::string urlOutgoingRequests)
    :ZmqProtobufNode( true ),
      m_repo( repo ),
      m_urlOutgoing( urlOutgoingRequests ),
      m_portIncoming( portIncomingRequests )
{
    void (mapit::ZmqResponderPrivate::*member)(google::protobuf::Message*);
    std::function<void(google::protobuf::Message*)> fn;

    member = &mapit::ZmqResponderPrivate::toDelegate<RequestWorkspace, &mapit::ZmqResponderPrivate::handleRequestWorkspace>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestWorkspace>( fn );

    member = &mapit::ZmqResponderPrivate::toDelegate<RequestEntitydata, &mapit::ZmqResponderPrivate::handleRequestEntitydata>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestEntitydata>( fn );

    member = &mapit::ZmqResponderPrivate::toDelegate<RequestHierarchy, &mapit::ZmqResponderPrivate::handleRequestHierarchy>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestHierarchy>( fn );

    member = &mapit::ZmqResponderPrivate::toDelegate<RequestHierarchyPlain, &mapit::ZmqResponderPrivate::handleRequestHierarchyPlain>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestHierarchyPlain>( fn );

    member = &mapit::ZmqResponderPrivate::toDelegate<RequestListWorkspaces, &mapit::ZmqResponderPrivate::handleRequestListWorkspaces>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestListWorkspaces>( fn );

    member = &mapit::ZmqResponderPrivate::toDelegate<RequestOperatorExecution, &mapit::ZmqResponderPrivate::handleRequestOperatorExecution>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestOperatorExecution>( fn );

    member = &mapit::ZmqResponderPrivate::toDelegate<RequestStoreOperatorExecution, &mapit::ZmqResponderPrivate::handleRequestStoreOperatorExecution>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestStoreOperatorExecution>( fn );

    member = &mapit::ZmqResponderPrivate::toDelegate<RequestGenericEntry, &mapit::ZmqResponderPrivate::handleRequestGenericEntry>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestGenericEntry>( fn );

    member = &mapit::ZmqResponderPrivate::toDelegate<RequestStoreEntity, &mapit::ZmqResponderPrivate::handleRequestStoreEntity>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestStoreEntity>( fn );

    member = &mapit::ZmqResponderPrivate::toDelegate<RequestDeleteEntity, &mapit::ZmqResponderPrivate::handleRequestDeleteEntity>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestDeleteEntity>( fn );

    member = &mapit::ZmqResponderPrivate::toDelegate<RequestDeleteTree, &mapit::ZmqResponderPrivate::handleRequestDeleteTree>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestDeleteTree>( fn );

    member = &mapit::ZmqResponderPrivate::toDelegate<RequestDoCommit, &mapit::ZmqResponderPrivate::handleRequestDoCommit>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestDoCommit>( fn );

    member = &mapit::ZmqResponderPrivate::toDelegate<RequestCommit, &mapit::ZmqResponderPrivate::handleRequestCommit>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestCommit>( fn );

    //TODO: allow ssh://?
    bind("tcp://*:" + std::to_string( m_portIncoming ) );
}

void mapit::ZmqResponderPrivate::handleRequestWorkspace(RequestWorkspace *msg)
{
    std::unique_ptr<ReplyWorkspace> ptr(new ReplyWorkspace());
    std::shared_ptr<Workspace> workspace = m_repo->getWorkspace(msg->workspace());
    if(workspace == NULL)
    {
        if(msg->createifnotexists())
        {
            //TODO: handle multiple commits for merge
            CommitId commit = msg->commit_size() == 0 ? "" : msg->commit(0);
            workspace = m_repo->createWorkspace(commit, msg->workspace());
            if( workspace == NULL )
            {
                log_info("Could not get workspace \"" + msg->workspace() + "\"");
                ptr->set_status( ReplyWorkspace::ERROR );
            }
            else
            {
                *ptr->mutable_commit() = msg->commit();
                ptr->set_status( ReplyWorkspace::SUCCESS );
            }
        }
        else
        {
            ptr->set_status( ReplyWorkspace::SUCCESS );
        }
    }
    else
    {
        std::vector<CommitId> ids(workspace->getParentCommitIds());
        for(std::vector<CommitId>::const_iterator citer(ids.cbegin()); citer != ids.cend() ; ++citer)
        {
            ptr->add_commit( *citer );
        }
        ptr->set_status( ReplyWorkspace::EXISTED );
    }
    send( std::move( ptr ) );
}

void mapit::ZmqResponderPrivate::handleRequestEntitydata(RequestEntitydata *msg)
{
    try {
        std::unique_ptr<ReplyEntitydata> ptr(new ReplyEntitydata());

        // Validate input
        std::shared_ptr<Workspace> workspace = m_repo->getWorkspace(msg->workspace());
        if(workspace == NULL)
        {
            ptr->set_status( ReplyEntitydata::NOT_FOUND );
            ptr->set_receivedlength( 0 );
            ptr->set_entitylength( 0 );
            send( std::move( ptr ) );
            return;
        }
        std::shared_ptr<AbstractEntitydata> ed = workspace->getEntitydataReadOnly( msg->entitypath() );

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
    //            status = mapit::ReplyEntitydata::EXCEEDED_BOUNDARY;
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

        mapit::uint64_t offset = msg->offset(), len = msg->maxlength();
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
        mapit::istream *strm = ed->startReadBytes( offset, len);
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

void mapit::ZmqResponderPrivate::handleRequestHierarchy(RequestHierarchy *msg)
{
    //TODO: Remove from service definition
    //Note that everything this message does can be done using internal messages "getTree" and "getEntity".
    std::unique_ptr<ReplyHierarchy> rep(new ReplyHierarchy());
    send(std::move(rep));
}

void mapit::ZmqResponderPrivate::handleRequestHierarchyPlain(RequestHierarchyPlain *msg)
{
    std::unique_ptr<ReplyHierarchyPlain> rep(new ReplyHierarchyPlain());
    std::shared_ptr<Workspace> workspace = m_repo->getWorkspace(msg->workspace());
    if(workspace == NULL)
    {
        // TODO: Introduce Error-type
        rep->set_status( ReplyHierarchyPlain::WORKSPACE_NOT_FOUND );
        send( std::move( rep ) );
    }
    else
    {
        StatusCode s = workspace->depthFirstSearch(
            [&](std::shared_ptr<Tree> obj, const ObjectReference &ref, const Path &path){return true;},
            [&](std::shared_ptr<Tree> obj, const ObjectReference &ref, const Path &path) {return true;},
            [&](std::shared_ptr<Entity> obj, const ObjectReference &ref, const Path &path)
            {
                rep->add_entities(path);
                return true;
            },
            [&](std::shared_ptr<Entity> obj, const ObjectReference &ref, const Path &path){return true;});
        if(!mapitIsOk(s))
        {
            log_info("error while listing entities (dfs)");
        }
    }
}

void mapit::ZmqResponderPrivate::handleRequestListWorkspaces(RequestListWorkspaces *msg)
{
    std::unique_ptr<ReplyListWorkspaces> rep(new ReplyListWorkspaces());
    std::vector<std::string> workspaces = m_repo->listWorkspaceNames();
    for(std::vector<std::string>::const_iterator iter(workspaces.cbegin()) ; iter != workspaces.cend() ; ++iter)
    {
        rep->add_workspaces(*iter);
    }
    send( std::move( rep ) );
}

void mapit::ZmqResponderPrivate::handleRequestOperatorExecution(RequestOperatorExecution *msg)
{
    std::unique_ptr<ReplyOperatorExecution> rep(new ReplyOperatorExecution());
    std::shared_ptr<Workspace> workspace = m_repo->getWorkspace(msg->workspace());
    if(workspace == NULL)
    {
        rep->set_status_code( MAPIT_STATUS_ERR_DB_NOT_FOUND );
        rep->set_error_msg( "Workspace does not exist" );
        send( std::move( rep ) );
        return;
    }
    mapit::OperationResult result = workspace->doOperation(msg->param());
    rep->set_status_code(result.first);
    rep->set_error_msg(""); // TODO: This is the success, errormessage. There are no more errormessages yet.
    send( std::move( rep ) );
}

void mapit::ZmqResponderPrivate::handleRequestStoreOperatorExecution(RequestStoreOperatorExecution* msg)
{
    std::unique_ptr<ReplyStoreOperatorExecution> rep(new ReplyStoreOperatorExecution());
    std::shared_ptr<Workspace> workspace = m_repo->getWorkspace(msg->workspace());
    if(workspace == NULL)
    {
        rep->set_status_code( MAPIT_STATUS_ERR_DB_NOT_FOUND );
        rep->set_error_msg( "Workspace does not exist" );
        send( std::move( rep ) );
        return;
    }

    workspace->storeOperationDesc_(msg->param(), false);

    rep->set_status_code(MAPIT_STATUS_OK);
    rep->set_error_msg("");
    send( std::move( rep ) );
}

void mapit::ZmqResponderPrivate::handleRequestStoreEntity(RequestStoreEntity *msg)
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
    std::shared_ptr<Workspace> workspace = m_repo->getWorkspace(msg->workspace());
    if(workspace == NULL)
    {
        discard_more();
        log_info("Workspace was not available");
        rep->set_status(ReplyStoreEntity::ERROR);
        send( std::move( rep ) );
        return;
    }
    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("StoreEntity");
    desc.set_params("{source:\"network\"}");

    mapit::OperationResult res = workspace->doUntraceableOperation(desc, [&msg, this](mapit::OperationEnvironment *env){
        mapit::operators::WorkspaceWritable* workspaceDirectWriteAccess = env->getWorkspace();

        // if offset is not 0, we assume the entity already exists.
        if(msg->offset() == 0)
        {
            // Receive Entity out of the message and compare to existing entity
            std::shared_ptr<Entity> entity(workspaceDirectWriteAccess->getEntity(msg->path()));
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
                return MAPIT_STATUS_INVALID_ARGUMENT;
            }
            else
            {
                // Other cases: There was a layertype and none is set now (just leave it as is).
                mapit::StatusCode status = workspaceDirectWriteAccess->storeEntity(msg->path(), entity);
                if(!mapitIsOk(status))
                {
                    log_info("Could not store entity: \"" + msg->path() + "\"");
                    return status;
                }
            }
        }
        if(msg->sendlength() == 0)
        {
            return MAPIT_STATUS_OK;
        }
        if(!has_more())
        {
            log_info("Server expected raw data (" + std::to_string(msg->sendlength()) + " bytes for entity \"" + msg->path() + "\"), but nothing was received.");
            return MAPIT_STATUS_INVALID_ARGUMENT;
        }
        // write entitydata
        std::shared_ptr<AbstractEntitydata> ed = workspaceDirectWriteAccess->getEntitydataForReadWrite(msg->path());
        if(ed == nullptr)
        {
            log_info("Entitydata of type could not be created.");
            return MAPIT_STATUS_ERR_MODULE_OPERATOR_NOT_FOUND;
        }
        mapit::ostream *stream = ed->startWriteBytes(msg->offset(), msg->sendlength());
        size_t offset = 0;
        while(has_more())
        {
            std::unique_ptr<zmq::message_t> buf( receive_raw_body() );
            if(buf->size() + offset > msg->sendlength())
            {
                log_info("Tried to store entitydata with sendlength smaller than received datasize.");
                ed->endWrite(stream);
                return MAPIT_STATUS_INVALID_ARGUMENT;
            }
            stream->write(static_cast<char*>(buf->data()), buf->size());
            offset += buf->size();
        }
        if(offset != msg->sendlength())
        {
            log_info("Received entitydata of wrong length: should: " + std::to_string(msg->sendlength()) + ", is: " + std::to_string(offset) + "." );
            ed->endWrite(stream);
            return MAPIT_STATUS_INVALID_DATA;
        }
        ed->endWrite(stream);
        return MAPIT_STATUS_OK;
    });
    if(mapitIsOk(res.first))
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

void
mapit::ZmqResponderPrivate::handleRequestDeleteEntity(RequestDeleteEntity* msg)
{
    std::unique_ptr<ReplyDeleteEntity> rep = std::make_unique<ReplyDeleteEntity>();
    std::shared_ptr<Workspace> workspace = m_repo->getWorkspace(msg->workspace());
    if(workspace == NULL)
    {
        discard_more();
        log_info("Workspace was not available");
        rep->set_status(ReplyDeleteEntity::ERROR);
        send( std::move( rep ) );
        return;
    }
    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("DeleteEntity");
    desc.set_params("{source:\"network\"}");

    mapit::OperationResult res = workspace->doUntraceableOperation(desc, [&msg, this](mapit::OperationEnvironment *env){
        mapit::operators::WorkspaceWritable* workspaceDirectWriteAccess = env->getWorkspace();

        return workspaceDirectWriteAccess->deleteEntity(msg->path());
    });
    if(mapitIsOk(res.first))
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
mapit::ZmqResponderPrivate::handleRequestDeleteTree(RequestDeleteTree* msg)
{
    std::unique_ptr<ReplyDeleteTree> rep = std::make_unique<ReplyDeleteTree>();
    std::shared_ptr<Workspace> workspace = m_repo->getWorkspace(msg->workspace());
    if(workspace == NULL)
    {
        discard_more();
        log_info("Workspace was not available");
        rep->set_status(ReplyDeleteTree::ERROR);
        send( std::move( rep ) );
        return;
    }
    OperationDescription desc;
    desc.mutable_operator_()->set_operatorname("DeleteTree");
    desc.set_params("{source:\"network\"}");

    mapit::OperationResult res = workspace->doUntraceableOperation(desc, [&msg, this](mapit::OperationEnvironment *env){
        mapit::operators::WorkspaceWritable* workspaceDirectWriteAccess = env->getWorkspace();

        return workspaceDirectWriteAccess->deleteTree(msg->path());
    });
    if(mapitIsOk(res.first))
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

void mapit::ZmqResponderPrivate::handleRequestGenericEntry(RequestGenericEntry *msg)
{
    Replier<ReplyGenericEntry> rep(new ReplyGenericEntry(), this);
    //std::unique_ptr<mapit::ReplyGenericEntry> rep(new mapit::ReplyGenericEntry());
    std::shared_ptr<Workspace> workspace = m_repo->getWorkspace(msg->workspace());
    if(workspace == nullptr)
    {
        log_info("Workspace for requestGenericEntry does not exist");
        rep.reply()->set_status( ReplyGenericEntry::NOT_FOUND );
        rep.send();
        return;
    }
    if (msg->path().empty()) { // empty means the rollingcommit
        std::shared_ptr<Commit> co = workspace->getRollingcommit();
        rep.reply()->set_status( ReplyGenericEntry::SUCCESS );
        rep.reply()->mutable_entry()->set_allocated_commit( new Commit( *co ) );
    } else {
        MessageType type = workspace->typeOfObject(msg->path());
        if(type == MessageTree)
        {
            std::shared_ptr<Tree> tree = workspace->getTree(msg->path());
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
            std::shared_ptr<Entity> entity = workspace->getEntity(msg->path());
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
    }
    rep.send();
}

void mapit::ZmqResponderPrivate::handleRequestDoCommit(RequestDoCommit* msg)
{
    std::unique_ptr<ReplyDoCommit> rep = std::make_unique<ReplyDoCommit>();

    std::shared_ptr<Workspace> workspace = m_repo->getWorkspace(msg->workspace());
    if (workspace == nullptr) {
        std::string error_msg = "Workspace \"" + msg->workspace() + "\"for RequestDoCommit does not exist";
        log_error(error_msg);
        rep->set_status_code(1);
        rep->set_error_msg( error_msg );
    } else {
        mapit::CommitId coID = m_repo->commit( workspace, msg->message(), msg->author(), msg->email(), mapit::time::from_msg(msg->stamp()) );
        rep->set_commit_id( coID );
        rep->set_status_code( 0 );
        rep->set_error_msg( "" );
    }

    send( std::move( rep ) );
}

void mapit::ZmqResponderPrivate::handleRequestCommit(RequestCommit* msg)
{
    std::unique_ptr<ReplyCommit> rep = std::make_unique<ReplyCommit>();

    std::shared_ptr<mapit::msgs::Commit> co = m_repo->getCommit( msg->commit_id() );
    if (co == nullptr) {
        std::string error_msg = "Commit \"" + msg->commit_id() + "\"for RequestCommit does not exist";
        log_info("handleRequestCommit: " + error_msg);
        rep->set_error_msg( error_msg );
        rep->set_status( ReplyCommit::ERROR );
    } else {
        rep->set_status( ReplyCommit::SUCCESS );
        *rep->mutable_commit() = *co.get();
    }

    send( std::move( rep ) );
}
