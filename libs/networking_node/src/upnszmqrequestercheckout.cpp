/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "upnszmqrequestercheckout.h"
#include <mapit/msgs/services_internal.pb.h>
#include <upns/serialization/entitydatalibrarymanager.h> //TODO: put in yet another independent project/cmake target. No dependecy to mapmanager required.
#include "serialization/zmqentitydatastreamprovider.h"
#include "operationenvironmentimpl.h"
#include <upns/errorcodes.h>
#include <upns/depthfirstsearch.h>
#include <upns/serialization/operatorlibrarymanager.h>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#include <upns/operators/module.h>
typedef ModuleInfo* (*GetModuleInfo)();

upns::ZmqRequesterCheckout::ZmqRequesterCheckout(std::string name, ZmqProtobufNode *node, Checkout *cache, bool operationsLocal)
    :m_checkoutName( name ),
     m_node( node ),
     m_cache( cache ),
     m_operationsLocal( operationsLocal )
{
}

bool upns::ZmqRequesterCheckout::isInConflictMode()
{
    //TODO: nyi
    assert(false);
    return false;
}

std::vector<std::shared_ptr<Conflict> > upns::ZmqRequesterCheckout::getPendingConflicts()
{
    //TODO: nyi
    assert(false);
    return std::vector<std::shared_ptr<Conflict> >();
}

void upns::ZmqRequesterCheckout::setConflictSolved(const upns::Path &path, const upns::ObjectId &oid)
{
    //TODO: nyi
    assert(false);
}

MessageType upns::ZmqRequesterCheckout::typeOfObject(const upns::Path &oidOrName)
{
    //TODO: Introduce typeof method with protobuf
    if(this->getTree(oidOrName) != nullptr) return MessageTree;
    if(this->getEntity(oidOrName) != nullptr) return MessageEntity;
    //if(this->get(oidOrName) != nullptr) return MessageCommit;
    //if(this->getTree(oidOrName) != nullptr) return MessageCheckout;
    //if(this->get(oidOrName) != nullptr) return MessageBranch;
    if(this->getEntitydataReadOnly(oidOrName) != nullptr) return MessageEntitydata;
    return MessageEmpty;
}

std::shared_ptr<Tree> upns::ZmqRequesterCheckout::getRoot()
{
    return getTree("/");
}

std::shared_ptr<Tree> upns::ZmqRequesterCheckout::getTreeConflict(const upns::ObjectId &objectId)
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<Tree>();
}

std::shared_ptr<Entity> upns::ZmqRequesterCheckout::getEntityConflict(const upns::ObjectId &objectId)
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<Entity>();
}

std::shared_ptr<Tree> upns::ZmqRequesterCheckout::getTree(const upns::Path &path)
{
    std::unique_ptr<RequestGenericEntry> req(new RequestGenericEntry);
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    try {
        m_node->prepareForwardComChannel();
        m_node->send(std::move(req));
        m_node->prepareBackComChannel();
        std::shared_ptr<ReplyGenericEntry> rep(m_node->receive<ReplyGenericEntry>());
        std::shared_ptr<Tree> ret(rep->mutable_entry()->release_tree());
        return ret;
    }
    catch (std::runtime_error err)
    {
        log_error("Server Error while getTree(" + path + "): " + err.what());
        return nullptr;
    }
    catch (zmq::error_t err)
    {
        log_error("Server Error while getTree(" + path + "): " + err.what());
        return nullptr;
    }
}

std::shared_ptr<Entity> upns::ZmqRequesterCheckout::getEntity(const upns::Path &path)
{
    std::unique_ptr<RequestGenericEntry> req(new RequestGenericEntry);
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    try {
        m_node->prepareForwardComChannel();
        m_node->send(std::move(req));
        m_node->prepareBackComChannel();
        std::shared_ptr<ReplyGenericEntry> rep(m_node->receive<ReplyGenericEntry>());
        std::shared_ptr<Entity> ret(rep->mutable_entry()->release_entity());
        return ret;
    }
    catch (std::runtime_error err)
    {
        log_error(err.what());
        return nullptr;
    }
}

std::shared_ptr<Branch> upns::ZmqRequesterCheckout::getParentBranch()
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<Branch>();
}

std::vector<upns::CommitId> upns::ZmqRequesterCheckout::getParentCommitIds()
{
    //TODO: nyi
    assert(false);
    return std::vector<upns::CommitId>();
}

std::shared_ptr<upns::AbstractEntitydata> upns::ZmqRequesterCheckout::getEntitydataReadOnly(const upns::Path &entityId)
{
    std::shared_ptr<Entity> e = getEntity(entityId);
    if(!e)
    {
        log_warn("Entity could not be queried for entitydata: " + entityId);
        return std::shared_ptr<upns::AbstractEntitydata>(nullptr);
    }
    std::shared_ptr<AbstractEntitydataProvider> streamProvider(new ZmqEntitydataStreamProvider(m_checkoutName, entityId, m_node));
    return EntityDataLibraryManager::getEntitydataFromProvider(e->type(), streamProvider);
}

std::shared_ptr<upns::AbstractEntitydata> upns::ZmqRequesterCheckout::getEntitydataReadOnlyConflict(const upns::ObjectId &entityId)
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<upns::AbstractEntitydata>(nullptr);
}

upns::StatusCode upns::ZmqRequesterCheckout::depthFirstSearch(  std::function<bool (std::shared_ptr<Tree>, const ObjectReference &, const Path &)> beforeTree
                                                              , std::function<bool (std::shared_ptr<Tree>, const ObjectReference &, const Path &)> afterTree
                                                              , std::function<bool (std::shared_ptr<Entity>, const ObjectReference &, const Path &)> beforeEntity
                                                              , std::function<bool (std::shared_ptr<Entity>, const ObjectReference &, const Path &)> afterEntity)
{
    return upns::depthFirstSearch(this, depthFirstSearchAll(mapit::msgs::Commit), depthFirstSearchAll(mapit::msgs::Commit), beforeTree, afterTree, beforeEntity, afterEntity);
}

upns::StatusCode upns::ZmqRequesterCheckout::depthFirstSearch(  const Path& path
                                                              , std::function<bool (std::shared_ptr<Tree>, const ObjectReference &, const Path &)> beforeTree
                                                              , std::function<bool (std::shared_ptr<Tree>, const ObjectReference &, const Path &)> afterTree
                                                              , std::function<bool (std::shared_ptr<Entity>, const ObjectReference &, const Path &)> beforeEntity
                                                              , std::function<bool (std::shared_ptr<Entity>, const ObjectReference &, const Path &)> afterEntity)
{
    return upns::depthFirstSearch(this, path, depthFirstSearchAll(mapit::msgs::Commit), depthFirstSearchAll(mapit::msgs::Commit), beforeTree, afterTree, beforeEntity, afterEntity);
}

upns::OperationResult upns::ZmqRequesterCheckout::doOperation(const OperationDescription &desc)
{
    if(m_operationsLocal)
    {
        // Execute operation on this machine. ZmqRequesterCheckoutRaw will read/write data from remote.
        return OperatorLibraryManager::doOperation(desc, this);
    }
    else
    {
        // Operator is executed on remote machine. Remote machine reads/writes however it is configured.
        std::unique_ptr<RequestOperatorExecution> req(new RequestOperatorExecution);
        req->set_checkout(m_checkoutName);
        *req->mutable_param() = desc;
        try {
            m_node->prepareForwardComChannel();
            m_node->send(std::move(req));
            m_node->prepareBackComChannel();
            std::shared_ptr<ReplyOperatorExecution> rep(m_node->receive<ReplyOperatorExecution>());
            upns::OperationResult res;
            res.first = rep->status_code();
            res.second = rep->result();
            if(!rep->error_msg().empty())
            {
                log_error("Remote Operator Executions returned an error: " + rep->error_msg());
            }
            return res;
        }
        catch (std::runtime_error err)
        {
            log_error(err.what());
            upns::OperationResult res;
            res.first = UPNS_STATUS_ERROR;
            return res;
        }
    }
}

upns::OperationResult upns::ZmqRequesterCheckout::doUntraceableOperation(const OperationDescription &desc, std::function<upns::StatusCode (upns::OperationEnvironment *)> operate)
{
    upns::OperationEnvironmentImpl env( desc );
    env.setCheckout( this );
    // TODO: this breaks req/resp pattern!
    upns::StatusCode status = operate( &env );
    OperationResult res(status, env.outputDescription());
    return res;
}

//void upns::ZmqRequesterCheckout::syncHierarchy()
//{
//    std::unique_ptr<upns::RequestHierarchy> req(new upns::RequestHierarchy);
//    req->set_checkout(m_checkoutName);
//    m_node->send(std::move(req)); catch
//    std::unique_ptr<upns::ReplyHierarchy> hierarchy(m_node->receive<upns::ReplyHierarchy>());
//    assert(m_cache);
//    //m_cache->
//}

upns::StatusCode upns::ZmqRequesterCheckout::storeTree(const upns::Path &path, std::shared_ptr<Tree> tree)
{
    return UPNS_STATUS_ERR_NOT_YET_IMPLEMENTED;
}

upns::StatusCode upns::ZmqRequesterCheckout::storeEntity(const upns::Path &path, std::shared_ptr<Entity> entity)
{
    std::unique_ptr<RequestStoreEntity> req(new RequestStoreEntity);
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    req->set_type( entity->type() );
    req->set_offset(0ul);
    req->set_sendlength(0ul);
    req->set_entitylength(0ul);
    try {
        m_node->prepareForwardComChannel();
        m_node->send(std::move(req));
        m_node->prepareBackComChannel();
        std::shared_ptr<ReplyStoreEntity> rep(m_node->receive<ReplyStoreEntity>());
        if(rep->status() == ReplyStoreEntity::SUCCESS)
        {
            return UPNS_STATUS_OK;
        }
        else
        {
            log_error("Could not store entity \"" + path + "\"");
            return UPNS_STATUS_ERROR;
        }
    }
    catch (std::runtime_error err)
    {
        log_error(err.what());
        return UPNS_STATUS_ERROR;
    }
}

upns::StatusCode upns::ZmqRequesterCheckout::deleteTree(const Path &path)
{
    std::unique_ptr<RequestDeleteTree> req = std::make_unique<RequestDeleteTree>();
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    m_node->send(std::move(req));

    std::shared_ptr<ReplyDeleteTree> rep(m_node->receive<ReplyDeleteTree>());
    if(rep->status() == ReplyDeleteTree::SUCCESS)
    {
        return UPNS_STATUS_OK;
    }
    else
    {
        log_error("ZmqRequesterCheckout: Could not delete tree \"" + path + "\"");
        return UPNS_STATUS_ERROR;
    }
}

upns::StatusCode upns::ZmqRequesterCheckout::deleteEntity(const Path &path)
{
    std::unique_ptr<RequestDeleteEntity> req = std::make_unique<RequestDeleteEntity>();
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    m_node->send(std::move(req));

    std::shared_ptr<ReplyDeleteEntity> rep(m_node->receive<ReplyDeleteEntity>());
    if(rep->status() == ReplyDeleteEntity::SUCCESS)
    {
        return UPNS_STATUS_OK;
    }
    else
    {
        log_error("ZmqRequesterCheckout: Could not delete entity \"" + path + "\"");
        return UPNS_STATUS_ERROR;
    }
}

std::shared_ptr<upns::AbstractEntitydata> upns::ZmqRequesterCheckout::getEntitydataForReadWrite(const upns::Path &entity)
{
    std::shared_ptr<Entity> e = getEntity(entity);
    if(!e)
    {
        log_error("Entity could not be queried for entitydata: " + entity);
        return std::shared_ptr<upns::AbstractEntitydata>(nullptr);
    }
    std::shared_ptr<AbstractEntitydataProvider> streamProvider(new ZmqEntitydataStreamProvider(m_checkoutName, entity, m_node));
    return EntityDataLibraryManager::getEntitydataFromProvider(e->type(), streamProvider);
}
