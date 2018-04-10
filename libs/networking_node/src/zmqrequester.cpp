/*******************************************************************************
 *
 * Copyright 2016-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "zmqrequester.h"
#include "zmqrequester_p.h"
#include "zmqrequesterworkspace.h"
#include <zmq.hpp>
#include <mapit/errorcodes.h>
#include <mutex>

mapit::ZmqRequester::ZmqRequester(Repository *cache, std::string urlOutgoingRequests, bool operationsLocal)
    :m_d( new mapit::ZmqRequesterPrivate( cache, urlOutgoingRequests, operationsLocal ) )
{

}

mapit::ZmqRequester::~ZmqRequester()
{
    delete m_d;
}

std::vector<std::string> mapit::ZmqRequester::listWorkspaceNames()
{
    std::unique_ptr<RequestListWorkspaces> req(new RequestListWorkspaces);
    try
    {
        m_d->m_requestMutex.lock();
        m_d->prepareForwardComChannel();
        m_d->send(std::move(req));
        m_d->prepareBackComChannel();
        std::shared_ptr<ReplyListWorkspaces> rep(m_d->receive<ReplyListWorkspaces>());
        m_d->m_requestMutex.unlock();

        std::vector<std::string> ret;
        if(rep == nullptr)
        {
            return ret;
        }

        ret.resize(rep->workspaces_size());
        for(int i=0 ; i<rep->workspaces_size() ; ++i)
        {
            ret.push_back(rep->workspaces(i));
        }
        return ret;
    }
    catch(zmq::error_t err)
    {
        m_d->m_requestMutex.unlock();
        log_error("ZmqRequester: Error in listWorkspaceNames: " + err.what());
        return std::vector<std::string>();
    }
}

std::shared_ptr<Tree> mapit::ZmqRequester::getTree(const mapit::ObjectId &oid)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

std::shared_ptr<Entity> mapit::ZmqRequester::getEntity(const mapit::ObjectId &oid)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

std::shared_ptr<Commit> mapit::ZmqRequester::getCommit(const mapit::CommitId &coID)
{
    std::unique_ptr<RequestCommit> req = std::make_unique<RequestCommit>();
    req->set_commit_id( coID );

    try {
        m_d->m_requestMutex.lock();
        m_d->prepareForwardComChannel();
        m_d->send(std::move(req));
        m_d->prepareBackComChannel();
        std::shared_ptr<ReplyCommit> rep(m_d->receive<ReplyCommit>());
        m_d->m_requestMutex.unlock();
        if(rep && (rep->status() == ReplyCommit::SUCCESS)) {
            return std::make_shared<Commit>( rep->commit() );
        } else {
            std::string errorMsg = "";
            if (rep == nullptr) {
                errorMsg = "no reply from server";
            } else {
                errorMsg = "with error \"" + rep->error_msg() + "\"";
            }
            log_error("Could not get commit \"" + coID + "\"\n" + errorMsg);
            return nullptr;
        }
    }
    catch(zmq::error_t err) {
        m_d->m_requestMutex.unlock();
        log_error("ZmqRequester: Error in getCommit: " + err.what());
        return nullptr;
    }
}

std::shared_ptr<WorkspaceObj> mapit::ZmqRequester::getWorkspaceObj(const std::string &name)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

std::shared_ptr<Branch> mapit::ZmqRequester::getBranch(const std::string &name)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

MessageType mapit::ZmqRequester::typeOfObject(const mapit::ObjectId &oid)
{
    //TODO: Define network message or use RequestGenericEntry
    if(this->getTree(oid) != nullptr) return MessageTree;
    if(this->getEntity(oid) != nullptr) return MessageEntity;
    if(this->getCommit(oid) != nullptr) return MessageCommit;
    if(this->getTree(oid) != nullptr) return MessageWorkspace;
    if(this->getBranch(oid) != nullptr) return MessageBranch;
    if(this->getEntitydataReadOnly(oid) != nullptr) return MessageEntitydata;
    return MessageEmpty;
}

std::shared_ptr<mapit::AbstractEntitydata> mapit::ZmqRequester::getEntitydataReadOnly(const mapit::ObjectId &oid)
{
    //TODO: Define network message
    //Locally cache whole object
    //Advanced feature: lazy
    return nullptr;
}

std::shared_ptr<mapit::Workspace> mapit::ZmqRequester::createWorkspace(const mapit::CommitId &commitIdOrBranchname, const std::string &name)
{
    std::unique_ptr<RequestWorkspace> req(new RequestWorkspace);
    req->set_workspace(name);
    req->add_commit(commitIdOrBranchname);
    req->set_createifnotexists(true);
    try
    {
        m_d->m_requestMutex.lock();
        m_d->prepareForwardComChannel();
        m_d->send(std::move(req));
        m_d->prepareBackComChannel();
        std::shared_ptr<ReplyWorkspace> rep(m_d->receive<ReplyWorkspace>());
        m_d->m_requestMutex.unlock();
        if(rep && (rep->status() == ReplyWorkspace::SUCCESS ||
           rep->status() == ReplyWorkspace::EXISTED))
        {
            return std::shared_ptr<mapit::Workspace>(new mapit::ZmqRequesterWorkspace( name, m_d, nullptr, m_d->m_operationsLocal, &m_d->m_requestMutex ));
        }
        else
        {
            log_error("Could not create workspace \"" + name + "\"");
            return std::shared_ptr<mapit::Workspace>(nullptr);
        }
    }
    catch(zmq::error_t err)
    {
        m_d->m_requestMutex.unlock();
        log_error("ZmqRequester: Error in createWorkspace: " + err.what());
        return std::shared_ptr<mapit::Workspace>(nullptr);
    }
}

std::shared_ptr<mapit::Workspace> mapit::ZmqRequester::getWorkspace(const std::string &workspaceName)
{
    //TODO: No error checking here at the time. It is possible, that the returned workspace does simply not exist.
    if(!m_d->m_operationsLocal)
    {
        // make sure remote repository is in sync with local
        // operator later must be able to compute locally without requests to client
        // TODO:
    }
    return std::shared_ptr<mapit::Workspace>(new mapit::ZmqRequesterWorkspace( workspaceName, m_d, nullptr, m_d->m_operationsLocal, &m_d->m_requestMutex ));
}

mapit::StatusCode mapit::ZmqRequester::deleteWorkspaceForced(const std::string &workspaceName)
{
    //TODO: nyi
    assert(false);
    return MAPIT_STATUS_ERR_NOT_YET_IMPLEMENTED;
}

mapit::CommitId mapit::ZmqRequester::commit(const std::shared_ptr<mapit::Workspace> workspace, std::string msg, std::string author, std::string email, mapit::time::Stamp stamp)
{
    std::unique_ptr<RequestDoCommit> req = std::make_unique<RequestDoCommit>();
    req->set_workspace( workspace->getName() );
    req->set_message( msg );
    req->set_author( author );
    req->set_email( email );
    req->set_allocated_stamp( mapit::time::to_msg_allocated( stamp ) );

    try {
        m_d->m_requestMutex.lock();
        m_d->prepareForwardComChannel();
        m_d->send(std::move(req));
        m_d->prepareBackComChannel();
        std::shared_ptr<ReplyDoCommit> rep(m_d->receive<ReplyDoCommit>());
        m_d->m_requestMutex.unlock();
        if(rep && (rep->status_code() == 0)) {
            return rep->commit_id();
        } else {
            std::string errorMsg = "";
            if (rep == nullptr) {
                errorMsg = "no reply from server";
            } else {
                errorMsg = "with error \"" + rep->error_msg() + "\"";
            }
            log_error("Could not commit \"" + workspace->getName() + "\"\n" + errorMsg);
            return "-1";
        }
    }
    catch(zmq::error_t err) {
        m_d->m_requestMutex.unlock();
        log_error("ZmqRequester: Error in commit: " + err.what());
        return "-1";
    }
}

std::vector<std::shared_ptr<Branch> > mapit::ZmqRequester::getBranches()
{
    //TODO: nyi
    assert(false);
    return std::vector<std::shared_ptr<Branch> >();
}

mapit::StatusCode mapit::ZmqRequester::push(mapit::Repository &repo)
{
    //TODO: nyi
    assert(false);
    return MAPIT_STATUS_ERR_NOT_YET_IMPLEMENTED;
}

mapit::StatusCode mapit::ZmqRequester::pull(mapit::Repository &repo)
{
    //TODO: nyi
    assert(false);
    return MAPIT_STATUS_ERR_NOT_YET_IMPLEMENTED;
}

mapit::CommitId mapit::ZmqRequester::parseCommitRef(const std::string &commitRef)
{
    //TODO: nyi
    assert(false);
    return commitRef;
}

std::shared_ptr<mapit::Workspace> mapit::ZmqRequester::merge(const mapit::CommitId mine, const mapit::CommitId theirs, const mapit::CommitId base)
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<mapit::Workspace>(nullptr);
}

std::vector<std::pair<mapit::CommitId, mapit::ObjectId> > mapit::ZmqRequester::ancestors(const mapit::CommitId &commitId, const mapit::ObjectId &objectId, const int level)
{
    //TODO: nyi
    assert(false);
    return std::vector<std::pair<mapit::CommitId, mapit::ObjectId> >();
}

bool mapit::ZmqRequester::canRead()
{
    return true;
}

bool mapit::ZmqRequester::canWrite()
{
    return true;
}
