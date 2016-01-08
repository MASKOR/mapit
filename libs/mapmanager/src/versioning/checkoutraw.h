#ifndef CHECKOUTRAW_H
#define CHECKOUTRAW_H

#include "upns_globals.h"
#include "services.pb.h"
#include "entitydata.h"

namespace upns
{

/**
 * @brief A Checkout object represents an editable state/version of all maps.
 * CheckoutRaw is the interface for Operators to directly edit objects.
 * Checkouts are also stored using AbstractMapSerializer. Changes can be done in a checkout without creating
 * new commits. Checkouts are serialized until they are commited.
 * When they are commited, the checkout gets transformed to the final commit. At this stage, it can not be changed anymore.
 * While not commited, checkouts behave like commits but one can not use the commit- and objectIds, because these are changing.
 * In contrast to all other places, objects behind the commit- and objectIds inside a checkout can be deleted from the system completely.
 * OperationDescriptors are collected while the checkout is active. Operations inside the list can have parameters (objectIds) which are
 * only temporary in the system. When replayed, dependencymanagement tries to compute all Operations which have all their parameters ready.
 */

class CheckoutRaw
{
public:
    StatusCode storeTree(ObjectId previous, Tree tree);
    StatusCode createTree(ObjectId parent, Tree tree);

    upnsSharedPointer<Entity> createEntity(ObjectId parent);

    void setConflictSolved(ObjectId solved);

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
    virtual upnsSharedPointer<AbstractEntityData> getEntityData(const ObjectId &entityId, bool readOnly = true) = 0;
};

}
#endif
