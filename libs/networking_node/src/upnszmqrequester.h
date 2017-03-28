#ifndef UPNSZMQREQUESTER_H
#define UPNSZMQREQUESTER_H

#include <string>
#include <upns/versioning/repository.h>

namespace upns {

class ZmqRequesterPrivate;

///
/// \brief The upns::ZmqRequester class
/// Implements the basic Repository Interface and will send requests over network
///

class ZmqRequester : public upns::Repository
{
public:
    ZmqRequester(Repository* cache, std::string urlOutgoingRequests = std::string(), bool operationsLocal = false);
    ~ZmqRequester();

    // Repository interface
public:
    std::vector<std::string> listCheckoutNames();
    std::shared_ptr<Tree> getTree(const ObjectId &oid);
    std::shared_ptr<Entity> getEntity(const ObjectId &oid);
    std::shared_ptr<Commit> getCommit(const ObjectId &oid);
    std::shared_ptr<CheckoutObj> getCheckoutObj(const std::string &name);
    std::shared_ptr<Branch> getBranch(const std::string &name);
    MessageType typeOfObject(const ObjectId &oid);
    std::shared_ptr<AbstractEntitydata> getEntitydataReadOnly(const ObjectId &oid);
    std::shared_ptr<Checkout> createCheckout(const CommitId &commitIdOrBranchname, const std::string &name);
    std::shared_ptr<Checkout> getCheckout(const std::string &checkoutName);
    StatusCode deleteCheckoutForced(const std::string &checkoutName);
    CommitId commit(const std::shared_ptr<Checkout> checkout, std::string msg);
    std::vector<std::shared_ptr<Branch> > getBranches();
    StatusCode push(Repository &repo);
    StatusCode pull(Repository &repo);
    CommitId parseCommitRef(const std::string &commitRef);
    std::shared_ptr<Checkout> merge(const CommitId mine, const CommitId theirs, const CommitId base);
    std::vector<std::pair<CommitId, ObjectId> > ancestors(const CommitId &commitId, const ObjectId &objectId, const int level);
    bool canRead();
    bool canWrite();

private:
    ZmqRequesterPrivate *m_d;
};

}

#endif // UPNSZMQREQUESTER_H
