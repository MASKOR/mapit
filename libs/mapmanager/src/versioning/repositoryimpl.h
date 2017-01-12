#ifndef REPOSITORYIMPL_H
#define REPOSITORYIMPL_H

#include "upns_typedefs.h"
#include "services.pb.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include "entitydata.h"
#include "versioning/checkout.h"
#include "versioning/repository.h"

namespace upns
{
class RepositoryPrivate;
class AbstractSerializer;
class RepositoryImpl : public Repository
{
public:
    RepositoryImpl(upns::upnsSharedPointer<upns::AbstractSerializer> serializer);
    virtual ~RepositoryImpl();

    upnsVec<upnsString> listCheckoutNames();

    upnsSharedPointer<Tree>         getTree(const ObjectId &oid);
    upnsSharedPointer<Entity>       getEntity(const ObjectId &oid);
    upnsSharedPointer<Commit>       getCommit(const ObjectId &oid);
    upnsSharedPointer<CheckoutObj>  getCheckoutObj(const upnsString &name);
    upnsSharedPointer<Branch>       getBranch(const upnsString &name);

    MessageType typeOfObject(const ObjectId &oid);

    upnsSharedPointer<AbstractEntitydata> getEntitydataReadOnly(const ObjectId &oid);

    upnsSharedPointer<Checkout> createCheckout(const CommitId &commitIdOrBranchname, const upnsString &name);
    upnsSharedPointer<Checkout> getCheckout(const upnsString &checkoutName);
    StatusCode                  deleteCheckoutForced(const upnsString &checkoutName);

    CommitId commit(const upnsSharedPointer<Checkout> checkout, upnsString msg);

    upnsVec< upnsSharedPointer<Branch> > getBranches();

    StatusCode push(Repository &repo);
    StatusCode pull(Repository &repo);

    CommitId parseCommitRef(const upnsString &commitRef);

    upnsSharedPointer<Checkout> merge(const CommitId mine, const CommitId theirs, const CommitId base);

    upnsVec< upnsPair<CommitId, ObjectId> > ancestors(const CommitId &commitId, const ObjectId &objectId, const int level = 0);

    bool canRead();
    bool canWrite();
private:
    RepositoryPrivate* m_p;
};

}
#endif
