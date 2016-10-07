#ifndef UPNSZMQREQUESTERCHECKOUT_H
#define UPNSZMQREQUESTERCHECKOUT_H

#include <string>
#include "versioning/repository.h"
#include "modules/versioning/checkoutraw.h"
#include "zmqnode.h"

namespace upns {

///
/// \brief The ZmqRequesterCheckout class
/// Implements the basic Checkout Interface and will send requests over network
///

class ZmqRequesterCheckout : public upns::Checkout, public upns::CheckoutRaw
{
public:
    ZmqRequesterCheckout(upnsString name, ZmqNode *node, upns::Checkout *cache = NULL);

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
    OperationResult doUntraceableOperation(const OperationDescription &desc, std::function<upns::StatusCode(upns::OperationEnvironment*)> operate);

    // CheckoutRaw interface
public:
    StatusCode storeTree(const Path &path, upnsSharedPointer<Tree> tree);
    StatusCode storeEntity(const Path &path, upnsSharedPointer<Entity> entity);
    upnsSharedPointer<AbstractEntityData> getEntityDataForReadWrite(const Path &entity);

private:
    upnsString m_checkoutName;
    ZmqNode *m_node;
    upns::Checkout *m_cache;

    void syncHierarchy();

};

}

#endif // UPNSZMQREQUESTER_H
