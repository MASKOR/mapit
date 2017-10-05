#ifndef CHECKOUTRAW_H
#define CHECKOUTRAW_H

#include <upns/typedefs.h>
#include <mapit/msgs/services.pb.h>
#include <upns/entitydata.h>
#include <upns/versioning/checkoutcommon.h>

namespace upns
{

/**
 * @brief A Checkout object represents an editable state/version of all maps.
 * CheckoutRaw is the interface for Operators to directly edit objects.
 * Conflicts:
 * When there is a conflict, a "path" is not enough to identify objects. An Operation,
 * working on conflicts can choose one of the existing versions or create a new object,
 * which will be marked as the new version.
 */

class CheckoutRaw : virtual public CheckoutCommon
{
protected:
    // Can not be deleted from outside (module)
    virtual ~CheckoutRaw() {}
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
    virtual StatusCode storeTree(const Path &path, std::shared_ptr<mapit::msgs::Tree> tree) = 0;

    /**
     * @brief storeEntity changes the entity at a given path. No conflict is generated.
     * The object at the given path must not be involved in a conflict.
     * @param path
     * @param tree
     * @return
     */
    virtual StatusCode storeEntity(const Path &path, std::shared_ptr<mapit::msgs::Entity> entity) = 0;

    /**
     * @brief getEntitydata Retrieves a data of the entity, which can be casted to a concrete type. Stream can be read maybe.
     * It can definitily not be read if entitydata is in conflict mode (Which one?).
     * It may be tracked, if the stream is accessed for read. Only then, the read data might be generated, if it is not stored in the cache.
     * If the stream is not read, Mapmanager can decide to overwrite the old stream directly and thus boost performance. However, operations which do not
     * read it's data may be rare / should be kept rare.
     * No conflict is generated, the old version is overwritten. If there was a conflict before, another candidate is added for this path/conflict.
     * @return
     */
    virtual std::shared_ptr<AbstractEntitydata> getEntitydataForReadWrite(const Path &entity) = 0;

    //TODO: deprecated, use stroreXXX
//    /**
//     * @brief createConflictSolvingTree creates a new tree at the given path. If there already is another tree/entity, a conflict is generated.
//     * Modules should only do this to resolve existing conflicts.
//     * @param path
//     * @return
//     */
//    virtual StatusCode createConflictSolvingTree(const Path &path, std::shared_ptr<Tree> tree) = 0;

//    /**
//     * @brief createEntity creates a new tree at the given path. If there already is another tree/entity, a conflict is generated.
//     * Marks the conflict solved.
//     * @param path
//     * @return
//     */
//    virtual StatusCode createConflictSolvingEntity(const Path &path, std::shared_ptr<Entity> entity) = 0;

    //TODO: deprecated, use above
//    /**
//     * @brief getEntitydataConflictingForWrite get one of the conflicting entitydatas, change it and mark it as the solution for the conflict.
//     * @param entity
//     * @return
//     */
//    virtual std::shared_ptr<AbstractEntitydata> getEntitydataConflictSolvingForWrite(const Path &entity) = 0;
};

}
#endif
