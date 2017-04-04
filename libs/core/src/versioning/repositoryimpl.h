#ifndef REPOSITORYIMPL_H
#define REPOSITORYIMPL_H

#include <upns/typedefs.h>
#include <mapit/msgs/services.pb.h>
#include <upns/operators/serialization/abstractentitydataprovider.h>
#include <upns/entitydata.h>
#include <upns/versioning/checkout.h>
#include <upns/versioning/repository.h>

namespace upns
{
class RepositoryPrivate;
class AbstractSerializer;
class RepositoryImpl : public Repository
{
public:
    RepositoryImpl(std::shared_ptr<upns::AbstractSerializer> serializer);
    virtual ~RepositoryImpl();

    std::vector<std::string> listCheckoutNames();

    std::shared_ptr<Tree>         getTree(const ObjectId &oid);
    std::shared_ptr<Entity>       getEntity(const ObjectId &oid);
    std::shared_ptr<Commit>       getCommit(const ObjectId &oid);
    std::shared_ptr<CheckoutObj>  getCheckoutObj(const std::string &name);
    std::shared_ptr<Branch>       getBranch(const std::string &name);

    MessageType typeOfObject(const ObjectId &oid);

    std::shared_ptr<AbstractEntitydata> getEntitydataReadOnly(const ObjectId &oid);

    std::shared_ptr<Checkout> createCheckout(const CommitId &commitIdOrBranchname, const std::string &name);
    std::shared_ptr<Checkout> getCheckout(const std::string &checkoutName);
    StatusCode                  deleteCheckoutForced(const std::string &checkoutName);

    CommitId commit(const std::shared_ptr<Checkout> checkout, std::string msg);

    std::vector< std::shared_ptr<Branch> > getBranches();

    StatusCode push(Repository &repo);
    StatusCode pull(Repository &repo);

    CommitId parseCommitRef(const std::string &commitRef);

    std::shared_ptr<Checkout> merge(const CommitId mine, const CommitId theirs, const CommitId base);

    std::vector< std::pair<CommitId, ObjectId> > ancestors(const CommitId &commitId, const ObjectId &objectId, const int level = 0);

    bool canRead();
    bool canWrite();
private:
    RepositoryPrivate* m_p;
};

}
#endif
