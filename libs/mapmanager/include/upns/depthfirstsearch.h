#ifndef DEPTHFIRSTSEARCH_H
#define DEPTHFIRSTSEARCH_H

#include <upns/typedefs.h>
#include <upns/logging.h>
#include "services.pb.h"
#include <upns/versioning/checkout.h>

namespace upns
{
/**
 * @brief Depth first search for Commit, Tree and Entity. This is often very handy.
 * Does not work for branches. Does not visit Entitydata (must be done manually).
 * If "before" returns false, "after" will not be executed.
 */
StatusCode depthFirstSearch(Checkout *checkout, upnsSharedPointer<Commit> obj, const ObjectReference &ref, const Path& path,
                            std::function<bool(upnsSharedPointer<Commit>, const ObjectReference&, const Path&)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectReference&, const Path&)> afterCommit,
                            std::function<bool(upnsSharedPointer<Tree>, const ObjectReference&, const Path&)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectReference&, const Path&)> afterTree,
                            std::function<bool(upnsSharedPointer<Entity>, const ObjectReference&, const Path&)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectReference&, const Path&)> afterEntity);

StatusCode depthFirstSearch(Checkout *checkout, upnsSharedPointer<Tree> obj, const ObjectReference &ref, const Path& path,
                            std::function<bool(upnsSharedPointer<Commit>, const ObjectReference&, const Path&)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectReference&, const Path&)> afterCommit,
                            std::function<bool(upnsSharedPointer<Tree>, const ObjectReference&, const Path&)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectReference&, const Path&)> afterTree,
                            std::function<bool(upnsSharedPointer<Entity>, const ObjectReference&, const Path&)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectReference&, const Path&)> afterEntity);

StatusCode depthFirstSearch(Checkout *checkout, upnsSharedPointer<Entity> obj, const ObjectReference &ref, const Path& path,
                            std::function<bool(upnsSharedPointer<Commit>, const ObjectReference&, const Path&)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectReference&, const Path&)> afterCommit,
                            std::function<bool(upnsSharedPointer<Tree>, const ObjectReference&, const Path&)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectReference&, const Path&)> afterTree,
                            std::function<bool(upnsSharedPointer<Entity>, const ObjectReference&, const Path&)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectReference&, const Path&)> afterEntity);

}

#endif
