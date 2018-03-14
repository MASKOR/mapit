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

#include "zmqrequestercheckout.h"
#include <mapit/msgs/services_internal.pb.h>
#include <mapit/serialization/entitydatalibrarymanager.h> //TODO: put in yet another independent project/cmake target. No dependecy to mapmanager required.
#include "serialization/zmqentitydatastreamprovider.h"
#include "operationenvironmentimpl.h"
#include <mapit/errorcodes.h>
#include <mapit/depthfirstsearch.h>
#include <mapit/serialization/operatorlibrarymanager.h>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#include <mapit/operators/module.h>
typedef ModuleInfo* (*GetModuleInfo)();

mapit::ZmqRequesterCheckout::ZmqRequesterCheckout(std::string name, ZmqProtobufNode *node, Checkout *cache, bool operationsLocal)
    :m_checkoutName( name ),
     m_node( node ),
     m_cache( cache ),
     m_operationsLocal( operationsLocal )
{
}

bool mapit::ZmqRequesterCheckout::isInConflictMode()
{
    //TODO: nyi
    assert(false);
    return false;
}

std::vector<std::shared_ptr<Conflict> > mapit::ZmqRequesterCheckout::getPendingConflicts()
{
    //TODO: nyi
    assert(false);
    return std::vector<std::shared_ptr<Conflict> >();
}

void mapit::ZmqRequesterCheckout::setConflictSolved(const mapit::Path &path, const mapit::ObjectId &oid)
{
    //TODO: nyi
    assert(false);
}

MessageType mapit::ZmqRequesterCheckout::typeOfObject(const mapit::Path &oidOrName)
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

std::shared_ptr<Tree> mapit::ZmqRequesterCheckout::getRoot()
{
    return getTree("/");
}

std::shared_ptr<Tree> mapit::ZmqRequesterCheckout::getTreeConflict(const mapit::ObjectId &objectId)
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<Tree>();
}

std::shared_ptr<Entity> mapit::ZmqRequesterCheckout::getEntityConflict(const mapit::ObjectId &objectId)
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<Entity>();
}

std::shared_ptr<Tree> mapit::ZmqRequesterCheckout::getTree(const mapit::Path &path)
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

std::shared_ptr<Entity> mapit::ZmqRequesterCheckout::getEntity(const mapit::Path &path)
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

std::shared_ptr<Branch> mapit::ZmqRequesterCheckout::getParentBranch()
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<Branch>();
}

std::vector<mapit::CommitId> mapit::ZmqRequesterCheckout::getParentCommitIds()
{
    //TODO: nyi
    assert(false);
    return std::vector<mapit::CommitId>();
}

std::shared_ptr<mapit::AbstractEntitydata> mapit::ZmqRequesterCheckout::getEntitydataReadOnly(const mapit::Path &entityId)
{
    std::shared_ptr<Entity> e = getEntity(entityId);
    if(!e)
    {
        log_warn("Entity could not be queried for entitydata: " + entityId);
        return std::shared_ptr<mapit::AbstractEntitydata>(nullptr);
    }
    std::shared_ptr<AbstractEntitydataProvider> streamProvider(new ZmqEntitydataStreamProvider(m_checkoutName, entityId, m_node));
    return EntityDataLibraryManager::getEntitydataFromProvider(e->type(), streamProvider);
}

std::shared_ptr<mapit::AbstractEntitydata> mapit::ZmqRequesterCheckout::getEntitydataReadOnlyConflict(const mapit::ObjectId &entityId)
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<mapit::AbstractEntitydata>(nullptr);
}

mapit::StatusCode mapit::ZmqRequesterCheckout::depthFirstSearch(  std::function<bool (std::shared_ptr<Tree>, const ObjectReference &, const Path &)> beforeTree
                                                              , std::function<bool (std::shared_ptr<Tree>, const ObjectReference &, const Path &)> afterTree
                                                              , std::function<bool (std::shared_ptr<Entity>, const ObjectReference &, const Path &)> beforeEntity
                                                              , std::function<bool (std::shared_ptr<Entity>, const ObjectReference &, const Path &)> afterEntity)
{
    return mapit::depthFirstSearch(this, depthFirstSearchAll(mapit::msgs::Commit), depthFirstSearchAll(mapit::msgs::Commit), beforeTree, afterTree, beforeEntity, afterEntity);
}

mapit::StatusCode mapit::ZmqRequesterCheckout::depthFirstSearch(  const Path& path
                                                              , std::function<bool (std::shared_ptr<Tree>, const ObjectReference &, const Path &)> beforeTree
                                                              , std::function<bool (std::shared_ptr<Tree>, const ObjectReference &, const Path &)> afterTree
                                                              , std::function<bool (std::shared_ptr<Entity>, const ObjectReference &, const Path &)> beforeEntity
                                                              , std::function<bool (std::shared_ptr<Entity>, const ObjectReference &, const Path &)> afterEntity)
{
    return mapit::depthFirstSearch(this, path, depthFirstSearchAll(mapit::msgs::Commit), depthFirstSearchAll(mapit::msgs::Commit), beforeTree, afterTree, beforeEntity, afterEntity);
}

mapit::OperationResult mapit::ZmqRequesterCheckout::doOperation(const OperationDescription &desc)
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
            mapit::OperationResult res;
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
            mapit::OperationResult res;
            res.first = MAPIT_STATUS_ERROR;
            return res;
        }
    }
}

mapit::OperationResult mapit::ZmqRequesterCheckout::doUntraceableOperation(const OperationDescription &desc, std::function<mapit::StatusCode (mapit::OperationEnvironment *)> operate)
{
    mapit::OperationEnvironmentImpl env( desc );
    env.setCheckout( this );
    // TODO: this breaks req/resp pattern!
    mapit::StatusCode status = operate( &env );
    OperationResult res(status, env.outputDescription());
    return res;
}

//void mapit::ZmqRequesterCheckout::syncHierarchy()
//{
//    std::unique_ptr<mapit::RequestHierarchy> req(new mapit::RequestHierarchy);
//    req->set_checkout(m_checkoutName);
//    m_node->send(std::move(req)); catch
//    std::unique_ptr<mapit::ReplyHierarchy> hierarchy(m_node->receive<mapit::ReplyHierarchy>());
//    assert(m_cache);
//    //m_cache->
//}

mapit::StatusCode mapit::ZmqRequesterCheckout::storeTree(const mapit::Path &path, std::shared_ptr<Tree> tree)
{
    return MAPIT_STATUS_ERR_NOT_YET_IMPLEMENTED;
}

mapit::StatusCode mapit::ZmqRequesterCheckout::storeEntity(const mapit::Path &path, std::shared_ptr<Entity> entity)
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
            return MAPIT_STATUS_OK;
        }
        else
        {
            log_error("Could not store entity \"" + path + "\"");
            return MAPIT_STATUS_ERROR;
        }
    }
    catch (std::runtime_error err)
    {
        log_error(err.what());
        return MAPIT_STATUS_ERROR;
    }
}

mapit::StatusCode mapit::ZmqRequesterCheckout::deleteTree(const Path &path)
{
    std::unique_ptr<RequestDeleteTree> req = std::make_unique<RequestDeleteTree>();
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    m_node->send(std::move(req));

    std::shared_ptr<ReplyDeleteTree> rep(m_node->receive<ReplyDeleteTree>());
    if(rep->status() == ReplyDeleteTree::SUCCESS)
    {
        return MAPIT_STATUS_OK;
    }
    else
    {
        log_error("ZmqRequesterCheckout: Could not delete tree \"" + path + "\"");
        return MAPIT_STATUS_ERROR;
    }
}

mapit::StatusCode mapit::ZmqRequesterCheckout::deleteEntity(const Path &path)
{
    std::unique_ptr<RequestDeleteEntity> req = std::make_unique<RequestDeleteEntity>();
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    m_node->send(std::move(req));

    std::shared_ptr<ReplyDeleteEntity> rep(m_node->receive<ReplyDeleteEntity>());
    if(rep->status() == ReplyDeleteEntity::SUCCESS)
    {
        return MAPIT_STATUS_OK;
    }
    else
    {
        log_error("ZmqRequesterCheckout: Could not delete entity \"" + path + "\"");
        return MAPIT_STATUS_ERROR;
    }
}

std::shared_ptr<mapit::AbstractEntitydata> mapit::ZmqRequesterCheckout::getEntitydataForReadWrite(const mapit::Path &entity)
{
    std::shared_ptr<Entity> e = getEntity(entity);
    if(!e)
    {
        log_error("Entity could not be queried for entitydata: " + entity);
        return std::shared_ptr<mapit::AbstractEntitydata>(nullptr);
    }
    std::shared_ptr<AbstractEntitydataProvider> streamProvider(new ZmqEntitydataStreamProvider(m_checkoutName, entity, m_node));
    return EntityDataLibraryManager::getEntitydataFromProvider(e->type(), streamProvider);
}
