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
#include "zmqrequestercheckout.h"
#include <zmq.hpp>
#include <mapit/errorcodes.h>

mapit::ZmqRequester::ZmqRequester(Repository *cache, std::string urlOutgoingRequests, bool operationsLocal)
    :m_d( new mapit::ZmqRequesterPrivate( cache, urlOutgoingRequests, operationsLocal ) )
{

}

mapit::ZmqRequester::~ZmqRequester()
{
    delete m_d;
}

std::vector<std::string> mapit::ZmqRequester::listCheckoutNames()
{
    std::unique_ptr<RequestListCheckouts> req(new RequestListCheckouts);
    try
    {
        m_d->prepareForwardComChannel();
        m_d->send(std::move(req));
        m_d->prepareBackComChannel();
        std::shared_ptr<ReplyListCheckouts> rep(m_d->receive<ReplyListCheckouts>());

        std::vector<std::string> ret;
        if(rep == nullptr)
        {
            return ret;
        }

        ret.resize(rep->checkouts_size());
        for(int i=0 ; i<rep->checkouts_size() ; ++i)
        {
            ret.push_back(rep->checkouts(i));
        }
        return ret;
    }
    catch(zmq::error_t err)
    {
        log_error("ZmqRequester: Error in listCheckoutNames: " + err.what());
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

std::shared_ptr<Commit> mapit::ZmqRequester::getCommit(const mapit::ObjectId &oid)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

std::shared_ptr<CheckoutObj> mapit::ZmqRequester::getCheckoutObj(const std::string &name)
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
    if(this->getTree(oid) != nullptr) return MessageCheckout;
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

std::shared_ptr<mapit::Checkout> mapit::ZmqRequester::createCheckout(const mapit::CommitId &commitIdOrBranchname, const std::string &name)
{
    std::unique_ptr<RequestCheckout> req(new RequestCheckout);
    req->set_checkout(name);
    req->add_commit(commitIdOrBranchname);
    req->set_createifnotexists(true);
    try
    {
        m_d->prepareForwardComChannel();
        m_d->send(std::move(req));
        m_d->prepareBackComChannel();
        std::shared_ptr<ReplyCheckout> rep(m_d->receive<ReplyCheckout>());
        if(rep && (rep->status() == ReplyCheckout::SUCCESS ||
           rep->status() == ReplyCheckout::EXISTED))
        {
            return std::shared_ptr<mapit::Checkout>(new mapit::ZmqRequesterCheckout( name, m_d, nullptr, m_d->m_operationsLocal ));
        }
        else
        {
            log_error("Could not create checkout \"" + name + "\"");
            return std::shared_ptr<mapit::Checkout>(nullptr);
        }
    }
    catch(zmq::error_t err)
    {
        log_error("ZmqRequester: Error in createCheckout: " + err.what());
        return std::shared_ptr<mapit::Checkout>(nullptr);
    }
}

std::shared_ptr<mapit::Checkout> mapit::ZmqRequester::getCheckout(const std::string &checkoutName)
{
    //TODO: No error checking here at the time. It is possible, that the returned checkout does simply not exist.
    if(!m_d->m_operationsLocal)
    {
        // make sure remote repository is in sync with local
        // operator later must be able to compute locally without requests to client
        // TODO:
    }
    return std::shared_ptr<mapit::Checkout>(new mapit::ZmqRequesterCheckout( checkoutName, m_d, nullptr, m_d->m_operationsLocal ));
}

mapit::StatusCode mapit::ZmqRequester::deleteCheckoutForced(const std::string &checkoutName)
{
    //TODO: nyi
    assert(false);
    return MAPIT_STATUS_ERR_NOT_YET_IMPLEMENTED;
}

mapit::CommitId mapit::ZmqRequester::commit(std::shared_ptr<mapit::Checkout>& checkout, std::string msg)
{
    //TODO: nyi
    assert(false);
    return "nyi";
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

std::shared_ptr<mapit::Checkout> mapit::ZmqRequester::merge(const mapit::CommitId mine, const mapit::CommitId theirs, const mapit::CommitId base)
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<mapit::Checkout>(nullptr);
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
