#ifndef CHECKOUTCOMMON_H
#define CHECKOUTCOMMON_H

#include "upns_typedefs.h"
#include "services.pb.h"
#include "abstractentitydata.h"
#include <functional>

namespace upns
{

/**
 * @brief The CheckoutCommon class contains read and common methods of "Checkout" and "CheckoutRaw".
 */

class CheckoutCommon
{
public:
    virtual ~CheckoutCommon() {}
    /**
     * @brief isInConflictMode After a try to merge, a checkout with conflicts can be generated.
     * @return
     */
    virtual bool isInConflictMode() = 0;

    /**
     * @brief getConflicts gets references to mine, theirs and base of all conflicting trees/entities after merge.
     * In the Checkout Tree, the objects are stored with the ObjectReference of "mine".
     * All conflicts, marked as solved are not returned.
     * @return
     */
    virtual upnsVec< upnsSharedPointer<Conflict> > getPendingConflicts() = 0;

    /**
     * @brief setConflictSolved Used to choose one blob for a given path. The chosen oid can also be a new one which
     * has been generated with create* by a "mergetool"-operation.
     * @param solved
     * @param oid
     */
    virtual void setConflictSolved(const Path &path, const ObjectId &oid) = 0;

    /**
     * @brief getRoot get Entry point to all objects of this commit.
     * @return
     */
    virtual upnsSharedPointer<Tree> getRoot() = 0;

    /**
     * @brief getTreeConflict gets a Tree from repository/checkout. Tree must be reachable from this checkout (descendant of <root>)
     * This must only be used for conflicting objects. Use getTree otherwise
     * @param objectId
     * @return child
     */
    virtual upnsSharedPointer<Tree> getTreeConflict(const ObjectId &objectId) = 0;

    /**
     * @brief getEntityConflict gets an Entity from repository/checkout. Entity must be reachable from this checkout (descendant of <root>)
     * This must only be used for conflicting objects. Use getEntity otherwise
     * @param objectId
     * @return child
     */
    virtual upnsSharedPointer<Entity> getEntityConflict(const ObjectId &objectId) = 0;

    /**
     * @brief getTree
     * @param path
     * @return
     */
    virtual upnsSharedPointer<Tree> getTree(const Path &path) = 0;

    /**
     * @brief getEntity
     * @param path
     * @return
     */
    virtual upnsSharedPointer<Entity> getEntity(const Path &path) = 0;

    /**
     * @brief getParentBranch
     * @return
     */
    virtual upnsSharedPointer<Branch> getParentBranch() = 0;

    /**
     * @brief getParentCommitIds. This is the parrent of the current rolling commit
     * @return
     */
    virtual upnsVec<CommitId> getParentCommitIds() = 0;

    /**
     * @brief getEntitydata Retrieves a data of the entity, which can be casted to a concrete type
     * After the internally used stream provider calls "endWrite()", the stream gets hashed and new ObjectIds are generated.
     * Entity::id -> hash of stream
     * Tree (layer) id -> child updated, rehash
     * Tree (map  ) id -> child updated, rehash
     * Tree (root ) id -> child updated, rehash
     * @param entityId
     * @return
     */
    virtual upnsSharedPointer<AbstractEntitydata> getEntitydataReadOnly(const Path &entityId) = 0;

    /**
     * @brief getEntitydataConflictingReadOnly because a path is not enough to identify a conflicting entitydata, this method is introduced.
     * @param entityId
     * @return
     */
    virtual upnsSharedPointer<AbstractEntitydata> getEntitydataReadOnlyConflict(const ObjectId &entityId) = 0;

    /**
     * @brief depthFirstSearch goes through all reachable elements with a DFS. If one of the callbacks returns false, all other descending
     * callbacks are skipped. If this happens in a "before" callback, also the "after" callback is skipped. TODO: Add proper skipping,
     * of all elements, after "false".
     * @param beforeCommit
     * @param afterCommit
     * @param beforeTree
     * @param afterTree
     * @param beforeEntity
     * @param afterEntity
     * @return
     */
    virtual StatusCode depthFirstSearch(std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> afterCommit,
                                std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> afterTree,
                                std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> afterEntity) = 0;

    virtual MessageType typeOfObject(const Path &oidOrName) = 0;
};

}
#endif
