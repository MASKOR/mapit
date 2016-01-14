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
     * @brief storeTree changes the tree at a given path. No conflict is generated, the old version is overwritten.
     * If there already was a conflict, a new sibling/candidate is created. Use setConflictSolved and choose an Object for the next version.
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
    virtual StatusCode storeEntity(const Path &path, upnsSharedPointer<Entity> tree) = 0;

    /**
     * @brief getEntityData Retrieves a data of the entity, which can be casted to a concrete type. Stream can be read maybe.
     * It can definitily not be read if entitydata is in conflict mode (Which one?).
     * It may be tracked, if the stream is accessed for read. Only then, the read data might be generated, if it is not stored in the cache.
     * If the stream is not read, Mapmanager can decide to overwrite the old stream directly and thus boost performance. However, operations which do not
     * read it's data may be rare / should be kept rare.
     * No conflict is generated, the old version is overwritten. If there was a conflict before, another candidate is added for this path/conflict.
     * @return
     */
    virtual upnsSharedPointer<AbstractEntityData> getEntityDataForReadWrite(const Path &entity) = 0;

    //TODO: deprecated, use stroreXXX
//    /**
//     * @brief createConflictSolvingTree creates a new tree at the given path. If there already is another tree/entity, a conflict is generated.
//     * Modules should only do this to resolve existing conflicts.
//     * @param path
//     * @return
//     */
//    virtual StatusCode createConflictSolvingTree(const Path &path, upnsSharedPointer<Tree> tree) = 0;

//    /**
//     * @brief createEntity creates a new tree at the given path. If there already is another tree/entity, a conflict is generated.
//     * Marks the conflict solved.
//     * @param path
//     * @return
//     */
//    virtual StatusCode createConflictSolvingEntity(const Path &path, upnsSharedPointer<Entity> entity) = 0;

    //TODO: deprecated, use above
//    /**
//     * @brief getEntityDataConflictingForWrite get one of the conflicting entitydatas, change it and mark it as the solution for the conflict.
//     * @param entity
//     * @return
//     */
//    virtual upnsSharedPointer<AbstractEntityData> getEntityDataConflictSolvingForWrite(const Path &entity) = 0;
};

}
#endif
