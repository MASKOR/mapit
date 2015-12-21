#ifndef MODULE_UTILITY_H
#define MODULE_UTILITY_H

#include "upns.h"
#include "operationenvironment.h"
#include "upns_interface/services.pb.h"
namespace upns
{

// Shared pointer because map is not owned by any parent protobuf object
upnsSharedPointer<Map> parseMap(const OperationEnvironment *env, const OperationParameter *param)
{
    if( param == NULL
        || param->mapval() == 0)
    {
        return NULL;
    }
    upnsSharedPointer<Map> map = env->mapServiceVersioned()->getMap(param->mapval());
    assert( map->id() != 0 );
    return map;
}

// Not a shared pointer because memory is managed by protobuf. Do NOT delete/free the retrieved pointer manually.
Layer* parseLayer(Map* map, const OperationParameter *param)
{
    if( param == NULL
        || param->layerval() == 0)
    {
        return NULL;
    }
    Layer* layer = NULL;
    for(int i=0; i < map->layers_size() ; ++i)
    {
        Layer *l = map->mutable_layers(i);
        if(l->id() == param->layerval())
        {
            layer = l;
            assert( layer->id() != 0 );
            break;
        }
    }
    return layer;
}

Entity* parseEntity(Layer* layer, const OperationParameter *param, bool retrieveOmittedSingleEntity = false)
{
    Entity *entity = NULL;
    if(param->entityval() == 0)
    {
        if( retrieveOmittedSingleEntity && layer->entities_size() == 1 )
        {
            entity = layer->mutable_entities(0);
            assert( entity->id() != 0 );
        }
    }
    else
    {
        for(int i=0; i < layer->entities_size() ; ++i)
        {
            Entity *e = layer->mutable_entities(i);
            if(e->id() == param->entityval())
            {
                entity = e;
                assert( entity->id() != 0 );
                break;
            }
        }
        if( entity == NULL )
        {
            return NULL;
        }
    }
    return entity;
}
}
#endif
