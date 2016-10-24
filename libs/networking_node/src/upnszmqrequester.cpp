#include "upnszmqrequester.h"
#include "upnszmqrequester_p.h"
#include "upnszmqrequestercheckout.h"
#include <zmq.hpp>
#include "upns_errorcodes.h"

upns::ZmqRequester::ZmqRequester(Repository *cache, upns::upnsString urlOutgoingRequests)
    :m_d( new upns::ZmqRequesterPrivate( cache, urlOutgoingRequests ) )
{

}

upns::ZmqRequester::~ZmqRequester()
{
    delete m_d;
}

upns::upnsVec<upns::upnsString> upns::ZmqRequester::listCheckoutNames()
{
    std::unique_ptr<upns::RequestListCheckouts> req(new upns::RequestListCheckouts);
    m_d->send(std::move(req));
    upns::upnsSharedPointer<upns::ReplyListCheckouts> rep(m_d->receive<upns::ReplyListCheckouts>());

    upns::upnsVec<upns::upnsString> ret(rep->checkouts_size());
    for(int i=0 ; i<rep->checkouts_size() ; ++i)
    {
        ret.push_back(rep->checkouts(i));
    }
    return ret;
}

upns::upnsSharedPointer<upns::Tree> upns::ZmqRequester::getTree(const upns::ObjectId &oid)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

upns::upnsSharedPointer<upns::Entity> upns::ZmqRequester::getEntity(const upns::ObjectId &oid)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

upns::upnsSharedPointer<upns::Commit> upns::ZmqRequester::getCommit(const upns::ObjectId &oid)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

upns::upnsSharedPointer<upns::CheckoutObj> upns::ZmqRequester::getCheckoutObj(const upns::upnsString &name)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

upns::upnsSharedPointer<upns::Branch> upns::ZmqRequester::getBranch(const upns::upnsString &name)
{
    //TODO: Define network message
    assert(false);
    return nullptr;
}

upns::MessageType upns::ZmqRequester::typeOfObject(const upns::ObjectId &oid)
{
    //TODO: Define network message
    assert(false);
    return upns::MessageEmpty;
}

upns::upnsSharedPointer<upns::AbstractEntitydata> upns::ZmqRequester::getEntitydataReadOnly(const upns::ObjectId &oid)
{
    //TODO: Define network message
    //Locally cache whole object
    //Advanced feature: lazy
    return nullptr;
}

upns::upnsSharedPointer<upns::Checkout> upns::ZmqRequester::createCheckout(const upns::CommitId &commitIdOrBranchname, const upns::upnsString &name)
{
    std::unique_ptr<upns::RequestCheckout> req(new upns::RequestCheckout);
    req->set_checkout(name);
    req->set_commit(commitIdOrBranchname);
    m_d->send(std::move(req));
    upns::upnsSharedPointer<upns::ReplyCheckout> rep(m_d->receive<upns::ReplyCheckout>());
    if(rep->status() == upns::ReplyCheckout::SUCCESS ||
       rep->status() == upns::ReplyCheckout::EXISTED)
    {
        return upns::upnsSharedPointer<upns::Checkout>(new upns::ZmqRequesterCheckout( name, m_d ));
    }
    else
    {
        log_error("Could not create checkout \"" + name + "\"");
        return upns::upnsSharedPointer<upns::Checkout>(nullptr);
    }
}

upns::upnsSharedPointer<upns::Checkout> upns::ZmqRequester::getCheckout(const upns::upnsString &checkoutName)
{
    //TODO: No error checking here at the time. It is possible, that the returned checkout does simply not exist.
    return upns::upnsSharedPointer<upns::Checkout>(new upns::ZmqRequesterCheckout( checkoutName, m_d ));
}

upns::StatusCode upns::ZmqRequester::deleteCheckoutForced(const upns::upnsString &checkoutName)
{
    //TODO: nyi
    assert(false);
    return UPNS_STATUS_ERR_NOT_YET_IMPLEMENTED;
}

upns::CommitId upns::ZmqRequester::commit(const upnsSharedPointer<upns::Checkout> checkout, upns::upnsString msg)
{
    //TODO: nyi
    assert(false);
    return "nyi";
}

upns::upnsVec<upns::upnsSharedPointer<upns::Branch> > upns::ZmqRequester::getBranches()
{
    //TODO: nyi
    assert(false);
    return upns::upnsVec<upns::upnsSharedPointer<upns::Branch> >();
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

upns::CommitId upns::ZmqRequester::parseCommitRef(const upns::upnsString &commitRef)
{
    //TODO: nyi
    assert(false);
    return commitRef;
}

upns::upnsSharedPointer<upns::Checkout> upns::ZmqRequester::merge(const upns::CommitId mine, const upns::CommitId theirs, const upns::CommitId base)
{
    //TODO: nyi
    assert(false);
    return upns::upnsSharedPointer<upns::Checkout>(nullptr);
}

upns::upnsVec<upns::upnsPair<upns::CommitId, upns::ObjectId> > upns::ZmqRequester::ancestors(const upns::CommitId &commitId, const upns::ObjectId &objectId, const int level)
{
    //TODO: nyi
    assert(false);
    return upns::upnsVec<upns::upnsPair<upns::CommitId, upns::ObjectId> >();
}

bool upns::ZmqRequester::canRead()
{
    return true;
}

bool upns::ZmqRequester::canWrite()
{
    return true;
}
