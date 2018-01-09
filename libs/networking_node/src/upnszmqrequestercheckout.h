#ifndef UPNSZMQREQUESTERCHECKOUT_H
#define UPNSZMQREQUESTERCHECKOUT_H

#include <string>
#include <upns/versioning/repository.h>
#include <upns/operators/versioning/checkoutraw.h>
#include "zmqprotobufnode.h"

namespace upns {

///
/// \brief The ZmqRequesterCheckout class
/// Implements the basic Checkout Interface and will send requests over network
/// Compute local:
/// - true: makes it possible to read from filesystem locally, and write to remote repository
/// - false: makes it possible to read from remote filesystem to remote repo
///

class ZmqRequesterCheckout : public upns::Checkout, public upns::CheckoutRaw
{
public:
    ZmqRequesterCheckout(std::string name, ZmqProtobufNode *node, upns::Checkout *cache = NULL, bool operationsLocal = false);

    // CheckoutCommon interface
public:
    bool isInConflictMode();
    std::vector<std::shared_ptr<Conflict> > getPendingConflicts();
    void setConflictSolved(const Path &path, const ObjectId &oid);
    virtual MessageType typeOfObject(const Path &oidOrName);
    std::shared_ptr<Tree> getRoot();
    std::shared_ptr<Tree> getTreeConflict(const ObjectId &objectId);
    std::shared_ptr<Entity> getEntityConflict(const ObjectId &objectId);
    std::shared_ptr<Tree> getTree(const Path &path);
    std::shared_ptr<Entity> getEntity(const Path &path);
    std::shared_ptr<Branch> getParentBranch();
    std::vector<CommitId> getParentCommitIds();
    std::shared_ptr<AbstractEntitydata> getEntitydataReadOnly(const Path &entityId);
    std::shared_ptr<AbstractEntitydata> getEntitydataReadOnlyConflict(const ObjectId &entityId);
    StatusCode depthFirstSearch(std::function<bool (std::shared_ptr<Commit>, const ObjectReference &, const Path &)> beforeCommit,
                                std::function<bool (std::shared_ptr<Commit>, const ObjectReference &, const Path &)> afterCommit,
                                std::function<bool (std::shared_ptr<Tree>, const ObjectReference &, const Path &)> beforeTree,
                                std::function<bool (std::shared_ptr<Tree>, const ObjectReference &, const Path &)> afterTree,
                                std::function<bool (std::shared_ptr<Entity>, const ObjectReference &, const Path &)> beforeEntity,
                                std::function<bool (std::shared_ptr<Entity>, const ObjectReference &, const Path &)> afterEntity);

    // Checkout interface
public:
    OperationResult doOperation(const OperationDescription &desc);
    OperationResult doUntraceableOperation(const OperationDescription &desc, std::function<upns::StatusCode(upns::OperationEnvironment*)> operate);

    // CheckoutRaw interface
public:
    StatusCode storeTree(const Path &path, std::shared_ptr<Tree> tree);
    StatusCode storeEntity(const Path &path, std::shared_ptr<Entity> entity);
    virtual StatusCode deleteTree(const Path &path);
    virtual StatusCode deleteEntity(const Path &path);
    std::shared_ptr<AbstractEntitydata> getEntitydataForReadWrite(const Path &entity);

private:
    std::string m_checkoutName;
    ZmqProtobufNode *m_node;
    upns::Checkout *m_cache;
    bool m_operationsLocal;

    //void syncHierarchy();

};

}

#endif // UPNSZMQREQUESTER_H
