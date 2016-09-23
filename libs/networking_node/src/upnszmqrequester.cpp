#include "upnszmqrequester.h"
#include "upnszmqrequester_p.h"
#include <zmq.hpp>

upns::ZmqRequester::ZmqRequester(std::__cxx11::string urlOutgoingRequests)
    :m_d( new upns::ZmqRequesterPrivate(urlOutgoingRequests) )
{
}

upns::ZmqRequester::~ZmqRequester()
{
    delete m_d;
}

upns::upnsVec<upns::upnsString> upns::ZmqRequester::listCheckoutNames()
{

}

upns::upnsSharedPointer<upns::Tree> upns::ZmqRequester::getTree(const upns::ObjectId &oid)
{

}

upns::upnsSharedPointer<upns::Entity> upns::ZmqRequester::getEntity(const upns::ObjectId &oid)
{

}

upns::upnsSharedPointer<upns::Commit> upns::ZmqRequester::getCommit(const upns::ObjectId &oid)
{

}

upns::upnsSharedPointer<upns::CheckoutObj> upns::ZmqRequester::getCheckoutObj(const upns::upnsString &name)
{

}

upns::upnsSharedPointer<upns::Branch> upns::ZmqRequester::getBranch(const upns::upnsString &name)
{

}

upns::MessageType upns::ZmqRequester::typeOfObject(const upns::ObjectId &oid)
{

}

upns::upnsSharedPointer<upns::AbstractEntityData> upns::ZmqRequester::getEntityDataReadOnly(const upns::ObjectId &oid)
{

}

upns::upnsSharedPointer<upns::Checkout> upns::ZmqRequester::createCheckout(const upns::CommitId &commitIdOrBranchname, const upns::upnsString &name)
{

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
