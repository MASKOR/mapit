#include "upnszmqrequestercheckout.h"

upns::ZmqRequesterCheckout::ZmqRequesterCheckout(ZmqNode *node)
    :m_pNode(node)
{

}

bool upns::ZmqRequesterCheckout::isInConflictMode()
{

}

upns::upnsVec<upns::upnsSharedPointer<upns::Conflict> > upns::ZmqRequesterCheckout::getPendingConflicts()
{

}

void upns::ZmqRequesterCheckout::setConflictSolved(const upns::Path &path, const upns::ObjectId &oid)
{

}

upns::upnsSharedPointer<upns::Tree> upns::ZmqRequesterCheckout::getRoot()
{

}

upns::upnsSharedPointer<upns::Tree> upns::ZmqRequesterCheckout::getTreeConflict(const upns::ObjectId &objectId)
{

}

upns::upnsSharedPointer<upns::Entity> upns::ZmqRequesterCheckout::getEntityConflict(const upns::ObjectId &objectId)
{

}

upns::upnsSharedPointer<upns::Tree> upns::ZmqRequesterCheckout::getTree(const upns::Path &path)
{

}

upns::upnsSharedPointer<upns::Entity> upns::ZmqRequesterCheckout::getEntity(const upns::Path &path)
{

}

upns::upnsSharedPointer<upns::Branch> upns::ZmqRequesterCheckout::getParentBranch()
{

}

upns::upnsVec<upns::CommitId> upns::ZmqRequesterCheckout::getParentCommitIds()
{

}

upns::upnsSharedPointer<upns::AbstractEntityData> upns::ZmqRequesterCheckout::getEntitydataReadOnly(const upns::Path &entityId)
{

}

upns::upnsSharedPointer<upns::AbstractEntityData> upns::ZmqRequesterCheckout::getEntitydataReadOnlyConflict(const upns::ObjectId &entityId)
{

}

upns::StatusCode upns::ZmqRequesterCheckout::depthFirstSearch(std::function<bool (upns::upnsSharedPointer<upns::Commit>, const upns::ObjectId &, const upns::Path &)> beforeCommit, std::function<bool (upns::upnsSharedPointer<upns::Commit>, const upns::ObjectId &, const upns::Path &)> afterCommit, std::function<bool (upns::upnsSharedPointer<upns::Tree>, const upns::ObjectId &, const upns::Path &)> beforeTree, std::function<bool (upns::upnsSharedPointer<upns::Tree>, const upns::ObjectId &, const upns::Path &)> afterTree, std::function<bool (upns::upnsSharedPointer<upns::Entity>, const upns::ObjectId &, const upns::Path &)> beforeEntity, std::function<bool (upns::upnsSharedPointer<upns::Entity>, const upns::ObjectId &, const upns::Path &)> afterEntity)
{

}

upns::OperationResult upns::ZmqRequesterCheckout::doOperation(const upns::OperationDescription &desc)
{

}
