#include "upnszmqrequester.h"
#include "upnszmqrequester_p.h"
#include "upnszmqrequestercheckout.h"
#include <zmq.hpp>
#include <upns/errorcodes.h>

upns::ZmqRequester::ZmqRequester(Repository *cache, std::string urlOutgoingRequests, bool operationsLocal)
    :m_d( new upns::ZmqRequesterPrivate( cache, urlOutgoingRequests, operationsLocal ) )
{

}

upns::ZmqRequester::~ZmqRequester()
{
    delete m_d;
}

std::vector<std::string> upns::ZmqRequester::listCheckoutNames()
{
    std::unique_ptr<upns::RequestListCheckouts> req(new upns::RequestListCheckouts);
    m_d->send(std::move(req));
    std::shared_ptr<upns::ReplyListCheckouts> rep(m_d->receive<upns::ReplyListCheckouts>());

    std::vector<std::string> ret(rep->checkouts_size());
    for(int i=0 ; i<rep->checkouts_size() ; ++i)
    {
        ret.push_back(rep->checkouts(i));
    }
    return ret;
}

std::shared_ptr<upns::Tree> upns::ZmqRequester::getTree(const upns::ObjectId &oid)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

std::shared_ptr<upns::Entity> upns::ZmqRequester::getEntity(const upns::ObjectId &oid)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

std::shared_ptr<upns::Commit> upns::ZmqRequester::getCommit(const upns::ObjectId &oid)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

std::shared_ptr<upns::CheckoutObj> upns::ZmqRequester::getCheckoutObj(const std::string &name)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

std::shared_ptr<upns::Branch> upns::ZmqRequester::getBranch(const std::string &name)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

upns::MessageType upns::ZmqRequester::typeOfObject(const upns::ObjectId &oid)
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

std::shared_ptr<upns::AbstractEntitydata> upns::ZmqRequester::getEntitydataReadOnly(const upns::ObjectId &oid)
{
    //TODO: Define network message
    //Locally cache whole object
    //Advanced feature: lazy
    return nullptr;
}

std::shared_ptr<upns::Checkout> upns::ZmqRequester::createCheckout(const upns::CommitId &commitIdOrBranchname, const std::string &name)
{
    std::unique_ptr<upns::RequestCheckout> req(new upns::RequestCheckout);
    req->set_checkout(name);
    req->add_commit(commitIdOrBranchname);
    req->set_createifnotexists(true);
    m_d->send(std::move(req));
    std::shared_ptr<upns::ReplyCheckout> rep(m_d->receive<upns::ReplyCheckout>());
    if(rep->status() == upns::ReplyCheckout::SUCCESS ||
       rep->status() == upns::ReplyCheckout::EXISTED)
    {
        return std::shared_ptr<upns::Checkout>(new upns::ZmqRequesterCheckout( name, m_d, nullptr, m_d->m_operationsLocal ));
    }
    else
    {
        log_error("Could not create checkout \"" + name + "\"");
        return std::shared_ptr<upns::Checkout>(nullptr);
    }
}

std::shared_ptr<upns::Checkout> upns::ZmqRequester::getCheckout(const std::string &checkoutName)
{
    //TODO: No error checking here at the time. It is possible, that the returned checkout does simply not exist.
    if(!m_d->m_operationsLocal)
    {
        // make sure remote repository is in sync with local
        // operator later must be able to compute locally without requests to client
        // TODO:
    }
    return std::shared_ptr<upns::Checkout>(new upns::ZmqRequesterCheckout( checkoutName, m_d, nullptr, m_d->m_operationsLocal ));
}

upns::StatusCode upns::ZmqRequester::deleteCheckoutForced(const std::string &checkoutName)
{
    //TODO: nyi
    assert(false);
    return UPNS_STATUS_ERR_NOT_YET_IMPLEMENTED;
}

upns::CommitId upns::ZmqRequester::commit(const std::shared_ptr<upns::Checkout> checkout, std::string msg)
{
    //TODO: nyi
    assert(false);
    return "nyi";
}

std::vector<std::shared_ptr<upns::Branch> > upns::ZmqRequester::getBranches()
{
    //TODO: nyi
    assert(false);
    return std::vector<std::shared_ptr<upns::Branch> >();
}

upns::StatusCode upns::ZmqRequester::push(upns::Repository &repo)
{
    //TODO: nyi
    assert(false);
    return UPNS_STATUS_ERR_NOT_YET_IMPLEMENTED;
}

upns::StatusCode upns::ZmqRequester::pull(upns::Repository &repo)
{
    //TODO: nyi
    assert(false);
    return UPNS_STATUS_ERR_NOT_YET_IMPLEMENTED;
}

upns::CommitId upns::ZmqRequester::parseCommitRef(const std::string &commitRef)
{
    //TODO: nyi
    assert(false);
    return commitRef;
}

std::shared_ptr<upns::Checkout> upns::ZmqRequester::merge(const upns::CommitId mine, const upns::CommitId theirs, const upns::CommitId base)
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<upns::Checkout>(nullptr);
}

std::vector<std::pair<upns::CommitId, upns::ObjectId> > upns::ZmqRequester::ancestors(const upns::CommitId &commitId, const upns::ObjectId &objectId, const int level)
{
    //TODO: nyi
    assert(false);
    return std::vector<std::pair<upns::CommitId, upns::ObjectId> >();
}

bool upns::ZmqRequester::canRead()
{
    return true;
}

bool upns::ZmqRequester::canWrite()
{
    return true;
}
