#ifndef CHECKOUTRAW_H
#define CHECKOUTRAW_H

#include "upns_globals.h"
#include "services.pb.h"
#include "entitydata.h"
#include "versioning/checkoutcommon.h"

namespace upns
{

/**
 * @brief A Checkout object represents an editable state/version of all maps.
 * CheckoutRaw is the interface for Operators to directly edit objects.
 * Checkouts are also stored using AbstractMapSerializer. Changes can be done in a checkout without creating
 * new commits. Checkouts are serialized until they are commited and do not only exist in RAM.
 * When they are commited, the checkout is transformed to the final commit. At this stage, it can not be changed anymore (all objects are immutable).
 * While not commited, checkouts behave like commits but one can not use the commit- and objectIds, because these are changing.
 * Checkouts are navigated using paths, using a "/" delimiter and mapping 1:1 the tree hierarchy.
 * In contrast to all other places, objects behind the commit- and objectIds inside a checkout can be deleted from the system completely.
 * OperationDescriptors are collected while the checkout is active. Operations inside the list can have parameters (objectIds) which are
 * only temporary in the system. When replayed, dependencymanagement tries to compute all Operations which have all their parameters ready.
 * Conflicts:
 * When there is a conflict, a "path" is not enough to identify objects. An Operation, working on conflicts can choose one of the existing versions or create a new object,
 * which will be marked as the new version.
 * For EntityData
 */

class CheckoutRaw : public CheckoutCommon
{
public:
    /**
     * @brief storeTree changes the tree at a given path. No conflict is generated.
     * If NULL is given as a parameter, the tree is deleted.
     * The object at the given path must not be involved in a conflict.
     * New trees can be created with this method.
     * @param path
     * @param tree
     * @return
     */
    virtual StatusCode storeTree(const Path &path, upnsSharedPointer<Tree> tree) = 0;

    /**
     * @brief storeEntity changes the entity at a given path. No conflict is generated.
     * The object at the given path must not be involved in a conflict.
     * @param path
     * @param tree
     * @return
     */
    virtual StatusCode storeEntity(const Path &path, upnsSharedPointer<Tree> tree) = 0;

    /**
     * @brief createConflictSolvingTree creates a new tree at the given path. If there already is another tree/entity, a conflict is generated.
     * Modules should only do this to resolve existing conflicts.
     * @param path
     * @return
     */
    virtual StatusCode createConflictSolvingTree(const Path &path, upnsSharedPointer<Tree> tree) = 0;

    /**
     * @brief createEntity creates a new tree at the given path. If there already is another tree/entity, a conflict is generated.
     * Marks the conflict solved.
     * @param path
     * @return
     */
    virtual StatusCode createConflictSolvingEntity(const Path &path, upnsSharedPointer<Entity> entity) = 0;

    /**
     * @brief setConflictSolved Used to choose one blob for a given path. The chosen oid can also be a new one which
     * has been generated with create* by a "mergetool"-operation.
     * @param solved
     * @param oid
     */
    virtual void setConflictSolved(const Path &solved, const ObjectId &oid) = 0;

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
    virtual upnsSharedPointer<AbstractEntityData> getEntityDataForWrite(const Path &entity) = 0;

    /**
     * @brief getEntityDataConflictingForWrite get one of the conflicting entitydatas, change it and mark it as the solution for the conflict.
     * @param entity
     * @return
     */
    virtual upnsSharedPointer<AbstractEntityData> getEntityDataConflictingSolvingForWrite(const Path &entity) = 0;
};

}
#endif
