#ifndef CHECKOUT_H
#define CHECKOUT_H

#include "upns_globals.h"
#include "services.pb.h"
#include "abstractentitydatastreamprovider.h"
#include "../serialization/abstractmapserializerNEW.h"
#include "entitydata.h"

namespace upns
{

/**
 * @brief A Checkout object represents a version of all maps.
 * Checkouts are also stored using AbstractMapSerializer. Changes can be done in a checkout without creating
 * new commits. Checkouts are serialized until they are commited.
 * When they are commited, the checkout gets transformed to the final commit. At this stage, it can not be changed anymore.
 * While not commited, checkouts behave like commits but one can not use the commit- and objectIds, because these are changing.
 * In contrast to all other places, objects behind the commit- and objectIds inside a checkout can be deleted from the system completely.
 * OperationDescriptors are collected while the checkout is active. Operations inside the list can have parameters (objectIds) which are
 * only temporary in the system. When replayed, dependencymanagement tries to compute all Operations which have all their parameters ready.
 */

class Checkout
{
public:
    /**
     * @brief Checkout Checkouts represent a
     * @param serializer
     * @param commitOrCheckoutId
     */
    Checkout(AbstractMapSerializer serializer, const CommitId commitOrCheckoutId);
    virtual ~Checkout() {};

    /**
     * @brief isInConflictMode After a try to merge, a checkout with conflicts can be generated.
     * @return
     */
    bool isInConflictMode();

    void setConflictSolved(ObjectId solved);

    /**
     * @brief getConflicts gets references to mine, theirs and base of all conflicting treeishs/entities after merge.
     * In the Checkout Tree, the objects are stored with the ObjectReference of "mine".
     * All conflicts, marked as solved are not returned.
     * @return
     */
    upnsVec< upnsSharedPointer<Conflict> > getPendingConflicts();

    upnsSharedPointer<Treeish> getRoot();
    /**
     * @brief getChild gets a Treeish from repository. Treeish must be reachable from this checkout (descendant of <root>)
     * @param objectId
     * @return child
     */
    upnsSharedPointer<Treeish> getChild(ObjectId objectId);

    /**
     * @brief getMap same as getChild, but can be used to make code more understandable
     * @param objectId
     * @return childMap
     */
    inline upnsSharedPointer<Treeish> getMap(ObjectId objectId) { return getChild( objectId ); }
    /**
     * @brief getLayer same as getChild, but can be used to make code more understandable
     * @param objectId
     * @return childLayer
     */
    inline upnsSharedPointer<Treeish> getLayer(ObjectId objectId) { return getChild( objectId ); }
    /**
     * @brief getEntity same as getChild, but can be used to make code more understandable
     * @param objectId
     * @return childEntity
     */
    inline upnsSharedPointer<Treeish> getEntity(ObjectId objectId) { return getChild( objectId ); }

    StatusCode storeTreeish(ObjectId previous, Treeish treeish);
    StatusCode createTreeish(ObjectId parent, Treeish treeish);

    upnsSharedPointer<Entity> createEntity(ObjectId parent);

    virtual bool canRead() = 0;
    virtual bool canWrite() = 0;

    /**
     * @brief getEntityData Retrieves a data of the entity, which can be casted to a concrete type
     * After the internally used stream provider calls "endWrite()", the stream gets hashed and new ObjectIds are generated.
     * Entity::id -> hash of stream
     * Treeish (layer) id -> child updated, rehash
     * Treeish (map  ) id -> child updated, rehash
     * Treeish (root ) id -> child updated, rehash
     * @param entityId
     * @return
     */
    upnsSharedPointer<AbstractEntityData> getEntityData(const ObjectId &entityId);
};

}
#endif
