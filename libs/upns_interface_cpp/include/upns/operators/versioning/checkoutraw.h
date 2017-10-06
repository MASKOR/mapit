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



    /**
     * @brief getEntityDataReadWrite get read and writable data of an entety
     * @param entity
     * @return
     */
    std::shared_ptr<AbstractEntitydata> getEntityDataReadWrite(std::shared_ptr<mapit::Entity> entity)
    {
        return getEntitydataForReadWrite(entity->getDataPath());
    }

    /**
     * @brief getNewMap returns a new map with the given name, returns nullptr when this map allready exists
     * @param name
     * @return
     */
    std::shared_ptr<mapit::Map> getNewMap(const std::string name)
    {
        std::shared_ptr<mapit::Map> map = getMap(name);
        if (map == nullptr) {
            // does not exists => create
            std::shared_ptr<mapit::msgs::Tree> tree = std::shared_ptr<mapit::msgs::Tree>(new mapit::msgs::Tree);
            map = std::shared_ptr<mapit::Map>(new mapit::Map(tree, name));

            return map;
        } else {
            // does exists => do nothing
            return nullptr;
        }
    }

    /**
     * @brief getNewLayer returns a new layer in the given map with the given name, returns nullptr when this layer allready exists
     * @param map
     * @param name
     * @return
     */
    std::shared_ptr<mapit::Layer> getNewLayer(std::shared_ptr<mapit::Map> map, const std::string name)
    {
        std::shared_ptr<mapit::Layer> layer = getLayer(map, name);
        if (layer == nullptr) {
            // does not exists => create
            std::shared_ptr<mapit::msgs::Tree> tree = std::shared_ptr<mapit::msgs::Tree>(new mapit::msgs::Tree);
            layer = std::shared_ptr<mapit::Layer>(new mapit::Layer(tree, name, map));

            return layer;
        } else {
            // does exists => do nothing
            return nullptr;
        }
    }

    /**
     * @brief getNewEntity returns a new entity in the given layer with the given name, returns nullptr when this entity allready exists
     * @param layer
     * @param name
     * @param type_name
     * @return
     */
    std::shared_ptr<mapit::Entity> getNewEntity(std::shared_ptr<mapit::Layer> layer, const std::string name, const std::string type_name)
    {
        std::shared_ptr<mapit::Entity> entity = getEntity(layer, name);
        if (entity == nullptr) {
            // does not exists => create
            std::shared_ptr<mapit::msgs::Entity> entity_tree = std::shared_ptr<mapit::msgs::Entity>(new mapit::msgs::Entity);
            entity_tree->set_type(type_name);
            entity = std::shared_ptr<mapit::Entity>(new mapit::Entity(entity_tree, name, layer));

            return entity;
        } else {
            // does exists => do nothing
            return nullptr;
        }
    }

    /**
     * @brief storeEntity stores the given entity and entity data to the mapit system
     * @param entity
     * @param entity_data_to_be_saved
     * @return
     */
    template<typename LayerDataType>
    StatusCode storeEntity(std::shared_ptr<mapit::Entity> entity, std::shared_ptr<LayerDataType> entity_data_to_be_saved)
    {
        storeEntity(entity->getDataPath(), entity->getEntity());
        // TODO data as well in one function
        std::shared_ptr<upns::Entitydata<LayerDataType>> entity_data = std::dynamic_pointer_cast<upns::Entitydata<LayerDataType>>(getEntityDataReadWrite(entity));
        if (entity_data == NULL) {
            // this means that the data was wrongly saved, error is not here!
            assert(false);
        }
        // TODO entety data stream manager oder so
        entity_data->setData(entity_data_to_be_saved, 0);
    }

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
