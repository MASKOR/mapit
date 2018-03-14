/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CHECKOUTRAW_H
#define CHECKOUTRAW_H

#include <mapit/typedefs.h>
#include <mapit/msgs/services.pb.h>
#include <mapit/entitydata.h>
#include <mapit/versioning/checkoutcommon.h>
#include <algorithm>

namespace mapit
{

/**
 * @brief A Checkout object represents an editable state/version of all maps.
 * CheckoutRaw is the interface for Operators to directly edit objects.
 * Conflicts:
 * When there is a conflict, a "path" is not enough to identify objects. An Operation,
 * working on conflicts can choose one of the existing versions or create a new object,
 * which will be marked as the new version.
 */

class CheckoutRaw : public virtual CheckoutCommon
{
protected:
    // Can not be deleted from outside (module)
    virtual ~CheckoutRaw() {}
public:
//    /**
//     * @brief storeTree changes the tree at a given path. No conflict is generated, the old version is overwritten.
//     * If there already was a conflict, a new sibling/candidate is created. Use setConflictSolved and choose an Object for the next version.
//     * If NULL is given as a parameter, the tree is deleted.
//     * The object at the given path must not be involved in a conflict.
//     * New trees can be created with this method.
//     * @param path
//     * @param tree
//     * @return
//     */
//    virtual StatusCode storeTree(const Path &path, std::shared_ptr<mapit::msgs::Tree> tree) = 0;

    /**
     * @brief storeEntity changes the entity at a given path. No conflict is generated.
     * The object at the given path must not be involved in a conflict.
     * @param path
     * @param tree
     * @return
     */
    virtual StatusCode storeEntity(const Path &path, std::shared_ptr<mapit::msgs::Entity> entity) = 0;

    /**
     * @brief deleteEntity delete the entity and entitydata
     * @param path
     * @return
     */
    virtual StatusCode deleteEntity(const Path &path) = 0;

    /**
     * @brief deleteTree deletes all entities and entitiesdata below this tree
     * @param path
     * @return
     */
    virtual StatusCode deleteTree(const Path &path) = 0;

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
