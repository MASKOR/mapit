#ifndef UPNSZMQREQUESTER_H
#define UPNSZMQREQUESTER_H

#include <string>
#include "versioning/repository.h"

namespace upns {

class ZmqRequesterPrivate;

///
/// \brief The upns::ZmqRequester class
/// Implements the basic Repository Interface and will send requests over network
/// Note: This is a quiet complex task and requires a private API over network.
/// A shallow copy of checkout is not enough here. Objects must be exchanged by hash completely.
///

class ZmqRequester : public upns::Repository
{
public:
    ZmqRequester(Repository* cache, upnsString urlOutgoingRequests = std::string());
    ~ZmqRequester();
private:
    ZmqRequesterPrivate *m_d;

    // Repository interface
public:
    upnsVec<upnsString> listCheckoutNames();
    upnsSharedPointer<Tree> getTree(const ObjectId &oid);
    upnsSharedPointer<Entity> getEntity(const ObjectId &oid);
    upnsSharedPointer<Commit> getCommit(const ObjectId &oid);
    upnsSharedPointer<CheckoutObj> getCheckoutObj(const upnsString &name);
    upnsSharedPointer<Branch> getBranch(const upnsString &name);
    MessageType typeOfObject(const ObjectId &oid);
    upnsSharedPointer<AbstractEntityData> getEntityDataReadOnly(const ObjectId &oid);
    upnsSharedPointer<Checkout> createCheckout(const CommitId &commitIdOrBranchname, const upnsString &name);
    upnsSharedPointer<Checkout> getCheckout(const upnsString &checkoutName);
    StatusCode deleteCheckoutForced(const upnsString &checkoutName);
    CommitId commit(const upnsSharedPointer<Checkout> checkout, upnsString msg);
    upnsVec<upnsSharedPointer<Branch> > getBranches();
    StatusCode push(Repository &repo);
    StatusCode pull(Repository &repo);
    CommitId parseCommitRef(const upnsString &commitRef);
    upnsSharedPointer<Checkout> merge(const CommitId mine, const CommitId theirs, const CommitId base);
    upnsVec<upnsPair<CommitId, ObjectId> > ancestors(const CommitId &commitId, const ObjectId &objectId, const int level);
    bool canRead();
    bool canWrite();
};

}

#endif // UPNSZMQREQUESTER_H
