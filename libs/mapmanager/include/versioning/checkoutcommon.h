#ifndef CHECKOUTCOMMON_H
#define CHECKOUTCOMMON_H

#include "upns_globals.h"
#include "services.pb.h"

namespace upns
{

/**
 * @brief The CheckoutCommon class contains read ans common methods of "Checkout" and "CheckoutRaw".
 * Note: Must not contain implementations to avoid "deadly diamond of death".
 */

class CheckoutCommon
{
public:

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
     * @brief getEntityData Retrieves a data of the entity, which can be casted to a concrete type
     * After the internally used stream provider calls "endWrite()", the stream gets hashed and new ObjectIds are generated.
     * Entity::id -> hash of stream
     * Tree (layer) id -> child updated, rehash
     * Tree (map  ) id -> child updated, rehash
     * Tree (root ) id -> child updated, rehash
     * @param entityId
     * @return
     */
    virtual upnsSharedPointer<AbstractEntityData> getEntityDataReadOnly(const Path &entityId) = 0;

    /**
     * @brief getEntityDataConflictingReadOnly because a path is not enough to identify a conflicting entitydata, this method is introduced.
     * @param entityId
     * @return
     */
    virtual upnsSharedPointer<AbstractEntityData> getEntityDataReadOnlyConflict(const ObjectId &entityId) = 0;
};

}
#endif