#ifndef CHECKOUTCOMMON_H
#define CHECKOUTCOMMON_H

#include "upns_globals.h"

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

    virtual upnsSharedPointer<Tree> getRoot() = 0;
    /**
     * @brief getChild gets a Tree from repository. Tree must be reachable from this checkout (descendant of <root>)
     * @param objectId
     * @return child
     */
    virtual upnsSharedPointer<Tree> getChild(ObjectId objectId) = 0;

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
    virtual upnsSharedPointer<AbstractEntityData> getEntityDataReadOnly(const ObjectId &entityId) = 0;
// TODO: remove these methods to not blow up implementation or make them virtual. (No Implementations must be here in this class)
//    /**
//     * @brief getMap same as getChild, but can be used to make code more understandable
//     * @param objectId
//     * @return childMap
//     */
//    inline upnsSharedPointer<Tree> getMap(ObjectId objectId) { return getChild( objectId ); }
//    /**
//     * @brief getLayer same as getChild, but can be used to make code more understandable
//     * @param objectId
//     * @return childLayer
//     */
//    inline upnsSharedPointer<Tree> getLayer(ObjectId objectId) { return getChild( objectId ); }
//    /**
//     * @brief getEntity same as getChild, but can be used to make code more understandable
//     * @param objectId
//     * @return childEntity
//     */
//    inline upnsSharedPointer<Tree> getEntity(ObjectId objectId) { return getChild( objectId ); }
};

}
#endif
