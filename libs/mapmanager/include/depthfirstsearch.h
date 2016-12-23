#ifndef DEPTHFIRSTSEARCH_H
#define DEPTHFIRSTSEARCH_H

#include "upns_typedefs.h"
#include "upns_logging.h"
#include "services.pb.h"
#include "versioning/checkout.h"

namespace upns
{
/**
 * @brief Depth first search for Commit, Tree and Entity.
 * Does not work for branches. Does not visit Entitydata (must be done manually).
 * If "before" returns false, "after" will not be executed.
 */
StatusCode depthFirstSearch(Checkout *checkout, upnsSharedPointer<Commit> obj, const ObjectId& oid, const Path &path,
                            std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> afterCommit,
                            std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> afterTree,
                            std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> afterEntity);

StatusCode depthFirstSearch(Checkout *checkout, upnsSharedPointer<Tree> obj, const ObjectId& oid, const Path &path,
                            std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> afterCommit,
                            std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> afterTree,
                            std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> afterEntity);

StatusCode depthFirstSearch(Checkout *checkout, upnsSharedPointer<Entity> obj, const ObjectId& oid, const Path &path,
                            std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> afterCommit,
                            std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> afterTree,
                            std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> afterEntity);

}

#endif
