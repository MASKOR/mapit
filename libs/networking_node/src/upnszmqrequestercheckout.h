#ifndef UPNSZMQREQUESTERCHECKOUT_H
#define UPNSZMQREQUESTERCHECKOUT_H

#include <string>
#include "versioning/repository.h"
#include "zmqnode.h"

namespace upns {

///
/// \brief The ZmqRequesterCheckout class
/// Implements the basic Checkout Interface and will send requests over network
///

class ZmqRequesterCheckout : public upns::Checkout
{
public:
    ZmqRequesterCheckout(ZmqNode* node);

    // CheckoutCommon interface
public:
    bool isInConflictMode();
    upnsVec<upnsSharedPointer<Conflict> > getPendingConflicts();
    void setConflictSolved(const Path &path, const ObjectId &oid);
    upnsSharedPointer<Tree> getRoot();
    upnsSharedPointer<Tree> getTreeConflict(const ObjectId &objectId);
    upnsSharedPointer<Entity> getEntityConflict(const ObjectId &objectId);
    upnsSharedPointer<Tree> getTree(const Path &path);
    upnsSharedPointer<Entity> getEntity(const Path &path);
    upnsSharedPointer<Branch> getParentBranch();
    upnsVec<CommitId> getParentCommitIds();
    upnsSharedPointer<AbstractEntityData> getEntitydataReadOnly(const Path &entityId);
    upnsSharedPointer<AbstractEntityData> getEntitydataReadOnlyConflict(const ObjectId &entityId);
    StatusCode depthFirstSearch(std::function<bool (upnsSharedPointer<Commit>, const ObjectId &, const Path &)> beforeCommit, std::function<bool (upnsSharedPointer<Commit>, const ObjectId &, const Path &)> afterCommit, std::function<bool (upnsSharedPointer<Tree>, const ObjectId &, const Path &)> beforeTree, std::function<bool (upnsSharedPointer<Tree>, const ObjectId &, const Path &)> afterTree, std::function<bool (upnsSharedPointer<Entity>, const ObjectId &, const Path &)> beforeEntity, std::function<bool (upnsSharedPointer<Entity>, const ObjectId &, const Path &)> afterEntity);

    // Checkout interface
public:
    OperationResult doOperation(const OperationDescription &desc);

private:
    ZmqNode *m_pNode;
};

}

#endif // UPNSZMQREQUESTER_H
