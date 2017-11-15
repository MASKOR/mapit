#ifndef CHECKOUTCOMMON_H
#define CHECKOUTCOMMON_H

#include <list>

#include <upns/typedefs.h>
#include <mapit/msgs/services.pb.h>
#include <upns/abstractentitydata.h>
#include <functional>

#include <mapit/versioning/map.h>
#include <mapit/versioning/layer.h>
#include <mapit/versioning/entity.h>

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
    virtual std::vector< std::shared_ptr<mapit::msgs::Conflict> > getPendingConflicts() = 0;

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
    virtual std::shared_ptr<mapit::msgs::Tree> getRoot() = 0;


    //TODO: This is another (api-)layer. Put code in a different place to hide none-map/layer/entity methods if this (api-)layer is used.
    // Keep map/layer/entity assumtions away from core, to not sacrifice ability to stream octrees and other structures.

    /**
     * @brief getMapPathOfEntry get top level tree of tree, map/layer/entity, path.
     * @return
     */
    static Path getMapPathOfEntry(const Path &path)
    {
        std::string p(path);
        while(p[0] == '/')
        {
            if(p.length() == 1) break;
            p = p.substr(1);
        }
        p = p.substr(0, p.find_first_of('/'));
        if(p.empty()) p = ".global";
        return p;
    }

    /**
     * @brief getListOfMaps get a list of all existing maps
     * @return
     */
    std::list<std::shared_ptr<mapit::Map>> getListOfMaps()
    {
        std::list<std::shared_ptr<mapit::Map>> maps;
        std::shared_ptr<mapit::msgs::Tree> root = getRoot();
        if (root) {
            for (const std::pair<std::string, mapit::msgs::ObjectReference> &ref_maps : root->refs()) {
//            for (auto ref_maps : getRoot()->refs()) {
                std::shared_ptr<mapit::msgs::Tree> map_tree = getTree( ref_maps.first );
                std::shared_ptr<mapit::Map> map = std::shared_ptr<mapit::Map>(new mapit::Map(map_tree, ref_maps.first));
                maps.push_back(map);
            }
        }

        return maps;
    }

    /**
     * @brief getMap get a map defined by the given name
     * @return
     */
    std::shared_ptr<mapit::Map> getMap(std::string map_name)
    {
        for (auto map : getListOfMaps()) {
            if (0 == map->getName().compare(map_name) ) {
                return map;
            }
        }
        return nullptr;
    }

    /**
     * @brief getListOfLayers get a list of all existing layers in the map
     * @param map
     * @return
     */
    std::list<std::shared_ptr<mapit::Layer>> getListOfLayers(std::shared_ptr<mapit::Map> map)
    {
        std::list<std::shared_ptr<mapit::Layer>> layers;
        for (auto ref_layers : map->getRefs()) {
            Path layerpath( map->getName() + "/" + ref_layers.first );
            std::shared_ptr<mapit::msgs::Tree> layer_tree = getTree( layerpath );
            std::string path_to_first_entity = layerpath + "/" + layer_tree->refs().begin()->first;
            std::string type = getEntity( path_to_first_entity )->type();
            std::shared_ptr<mapit::Layer> layer = std::shared_ptr<mapit::Layer>(new mapit::Layer(layer_tree, ref_layers.first, map, type));
            layers.push_back(layer);
        }
        return layers;
    }

    /**
     * @brief getLayer get the layer defined by the given name within the given map
     * @param map
     * @param layer_name
     * @return
     */
    std::shared_ptr<mapit::Layer> getLayer(std::shared_ptr<mapit::Map> map, std::string layer_name)
    {
        for (auto layer : getListOfLayers(map)) {
            if (0 == layer->getName().compare(layer_name) ) {
                return layer;
            }
        }
        return nullptr;
    }

    /**
     * @brief getListOfEntities get a list of all existing entities in the layer
     * @param layer
     * @return
     */
    std::list<std::shared_ptr<mapit::Entity>> getListOfEntities(std::shared_ptr<mapit::Layer> layer)
    {
        std::list<std::shared_ptr<mapit::Entity>> entities;
        for (auto ref_entities : layer->getRefs()) {
            std::shared_ptr<mapit::msgs::Entity> entity_tree = getEntity( layer->getDataPath() + ref_entities.first );
            std::shared_ptr<mapit::Entity> entity = std::shared_ptr<mapit::Entity>(new mapit::Entity(entity_tree, ref_entities.first, layer));
            entities.push_back(entity);
        }
        return entities;
    }

    /**
     * @brief getEntity get the entity defined by the given name within the given layer
     * @param layer
     * @param entity_name
     * @return
     */
    std::shared_ptr<mapit::Entity> getEntity(std::shared_ptr<mapit::Layer> layer, std::string entity_name)
    {
        // fast way
        auto entity_it = layer->getRefs().find(entity_name);
        if ( entity_it != layer->getRefs().end() ) {
            std::shared_ptr<mapit::msgs::Entity> entity_tree = getEntity( entity_it->second.path() );
            return std::shared_ptr<mapit::Entity>(new mapit::Entity(entity_tree, entity_it->first, layer));
//          return std::shared_ptr<mapit::Entity>();
        }

        return nullptr;

        // elegant way
//        for (auto entity : getListOfEntities(layer)) {
//            if (0 == entity->getName().compare(entity_name) ) {
//                return entity;
//            }
//        }
//        return nullptr;
    }

    /**
     * @brief getEntitydataReadOnly gets the data assosiated with the entity
     * @param entity
     * @return
     */
    std::shared_ptr<upns::AbstractEntitydata> getEntitydataReadOnly(std::shared_ptr<mapit::Entity> entity)
    {
        return getEntitydataReadOnly(entity->getDataPath());
    }

    /**
     * @brief getTreeConflict gets a Tree from repository/checkout. Tree must be reachable from this checkout (descendant of <root>)
     * This must only be used for conflicting objects. Use getTree otherwise
     * @param objectId
     * @return child
     */
    virtual std::shared_ptr<mapit::msgs::Tree> getTreeConflict(const ObjectId &objectId) = 0;

    /**
     * @brief getEntityConflict gets an Entity from repository/checkout. Entity must be reachable from this checkout (descendant of <root>)
     * This must only be used for conflicting objects. Use getEntity otherwise
     * @param objectId
     * @return child
     */
    virtual std::shared_ptr<mapit::msgs::Entity> getEntityConflict(const ObjectId &objectId) = 0;

    /**
     * @brief getTree
     * @param path
     * @return
     */
    virtual std::shared_ptr<mapit::msgs::Tree> getTree(const Path &path) = 0;

    /**
     * @brief getEntity
     * @param path
     * @return
     */
    virtual std::shared_ptr<mapit::msgs::Entity> getEntity(const Path &path) = 0;

    /**
     * @brief getParentBranch
     * @return
     */
    virtual std::shared_ptr<mapit::msgs::Branch> getParentBranch() = 0;

    /**
     * @brief getParentCommitIds. This is the parrent of the current rolling commit
     * @return
     */
    virtual std::vector<CommitId> getParentCommitIds() = 0;

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
    virtual std::shared_ptr<AbstractEntitydata> getEntitydataReadOnly(const Path &entityId) = 0;

    /**
     * @brief getEntitydataConflictingReadOnly because a path is not enough to identify a conflicting entitydata, this method is introduced.
     * @param entityId
     * @return
     */
    virtual std::shared_ptr<AbstractEntitydata> getEntitydataReadOnlyConflict(const ObjectId &entityId) = 0;

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
    virtual StatusCode depthFirstSearch(std::function<bool(std::shared_ptr<mapit::msgs::Commit>, const mapit::msgs::ObjectReference&, const Path&)> beforeCommit, std::function<bool(std::shared_ptr<mapit::msgs::Commit>, const mapit::msgs::ObjectReference&, const Path&)> afterCommit,
                                        std::function<bool(std::shared_ptr<mapit::msgs::Tree>, const mapit::msgs::ObjectReference&, const Path&)> beforeTree, std::function<bool(std::shared_ptr<mapit::msgs::Tree>, const mapit::msgs::ObjectReference&, const Path&)> afterTree,
                                        std::function<bool(std::shared_ptr<mapit::msgs::Entity>, const mapit::msgs::ObjectReference&, const Path&)> beforeEntity, std::function<bool(std::shared_ptr<mapit::msgs::Entity>, const mapit::msgs::ObjectReference&, const Path&)> afterEntity) = 0;

    virtual mapit::msgs::MessageType typeOfObject(const Path &oidOrName) = 0;
};

}
#endif
