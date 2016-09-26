#include "upnszmqrequester.h"
#include "upnszmqrequester_p.h"
#include "upnszmqrequestercheckout.h"
#include <zmq.hpp>

upns::ZmqRequester::ZmqRequester(Repository *cache, std::__cxx11::string urlOutgoingRequests)
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
    upns::ReplyListCheckouts *rep = m_d->receive<upns::ReplyListCheckouts>();

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
//    std::unique_ptr<upns::RequestHierarchy> req(new upns::RequestHierarchy);
//    req->
//    m_d->send(std::move(req));
//    upns::ReplyHierarchy *rep = m_d->receive<upns::ReplyHierarchy>();

//    upns::upnsVec<upns::upnsString> ret(rep->);
//    for(int i=0 ; i<rep->checkouts_size() ; ++i)
//    {
//        ret.push_back(rep->checkouts(i));
//    }
//    return ret;
    return nullptr;
}

upns::upnsSharedPointer<upns::Entity> upns::ZmqRequester::getEntity(const upns::ObjectId &oid)
{
    //TODO: Define network message
}

upns::upnsSharedPointer<upns::Commit> upns::ZmqRequester::getCommit(const upns::ObjectId &oid)
{
    //TODO: Define network message
}

upns::upnsSharedPointer<upns::CheckoutObj> upns::ZmqRequester::getCheckoutObj(const upns::upnsString &name)
{
    //TODO: Define network message
}

upns::upnsSharedPointer<upns::Branch> upns::ZmqRequester::getBranch(const upns::upnsString &name)
{
    //TODO: Define network message
}

upns::MessageType upns::ZmqRequester::typeOfObject(const upns::ObjectId &oid)
{
    //TODO: Define network message
}

upns::upnsSharedPointer<upns::AbstractEntityData> upns::ZmqRequester::getEntityDataReadOnly(const upns::ObjectId &oid)
{
    //TODO: Define network message
    //Locally cache whole object
    //Advanced feature: lazy
}

upns::upnsSharedPointer<upns::Checkout> upns::ZmqRequester::createCheckout(const upns::CommitId &commitIdOrBranchname, const upns::upnsString &name)
{
    std::unique_ptr<upns::RequestCheckout> req(new upns::RequestCheckout);
    m_d->send(std::move(req));
    upns::ReplyCheckout *rep = m_d->receive<upns::ReplyCheckout>();

    return upns::upnsSharedPointer<upns::Checkout>(new upns::ZmqRequesterCheckout( name, m_d ));
}

upns::upnsSharedPointer<upns::Checkout> upns::ZmqRequester::getCheckout(const upns::upnsString &checkoutName)
{

}

upns::StatusCode upns::ZmqRequester::deleteCheckoutForced(const upns::upnsString &checkoutName)
{

}

upns::CommitId upns::ZmqRequester::commit(const upnsSharedPointer<upns::Checkout> checkout, upns::upnsString msg)
{

}

upns::upnsVec<upns::upnsSharedPointer<upns::Branch> > upns::ZmqRequester::getBranches()
{

}

upns::StatusCode upns::ZmqRequester::push(upns::Repository &repo)
{

}

upns::StatusCode upns::ZmqRequester::pull(upns::Repository &repo)
{

}

upns::CommitId upns::ZmqRequester::parseCommitRef(const upns::upnsString &commitRef)
{

}

upns::upnsSharedPointer<upns::Checkout> upns::ZmqRequester::merge(const upns::CommitId mine, const upns::CommitId theirs, const upns::CommitId base)
{

}

upns::upnsVec<upns::upnsPair<upns::CommitId, upns::ObjectId> > upns::ZmqRequester::ancestors(const upns::CommitId &commitId, const upns::ObjectId &objectId, const int level)
{

}

bool upns::ZmqRequester::canRead()
{

}

bool upns::ZmqRequester::canWrite()
{

}
