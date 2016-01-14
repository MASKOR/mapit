#ifndef MODULE_UTILITY_H
#define MODULE_UTILITY_H

#include "upns.h"
#include "operationenvironment.h"
#include "libs/upns_interface/services.pb.h"
#include "modules/versioning/checkoutraw.h"

namespace upns
{

upnsSharedPointer<Tree> parseMap(const OperationEnvironment *env, const OperationParameter *param)
{
    //TODO: handle path or oid
    if( param == NULL
        || param->objectid() == "")
    {
        return NULL;
    }
    upnsSharedPointer<Tree> map = env->getCheckout()->getTree(param->objectid());
    assert( map != NULL );
    return map;
}

upnsSharedPointer<Tree> parseLayer(const OperationEnvironment *env, const OperationParameter *param)
{
    return upnsSharedPointer<Tree>(NULL);
    //TODO: handle path or oid
//    if( param == NULL
//            || param->objectid() == "")
//    {
//        return NULL;
//    }
//    Layer* layer = NULL;
//    for(int i=0; i < map->layers_size() ; ++i)
//    {
//        Layer *l = map->mutable_layers(i);
//        if(l->id() == param->layerval())
//        {
//            layer = l;
//            assert( layer->id() != 0 );
//            break;
//        }
//    }
}

upnsSharedPointer<Entity> parseEntity(const OperationEnvironment *env, const OperationParameter *param, bool retrieveOmittedSingleEntity = false)
{
    return upnsSharedPointer<Entity>(NULL);
//    Entity *entity = NULL;
//    if(param->entityval() == 0)
//    {
//        if( retrieveOmittedSingleEntity && layer->entities_size() == 1 )
//        {
//            entity = layer->mutable_entities(0);
//            assert( entity->id() != 0 );
//        }
//    }
//    else
//    {
//        for(int i=0; i < layer->entities_size() ; ++i)
//        {
//            Entity *e = layer->mutable_entities(i);
//            if(e->id() == param->entityval())
//            {
//                entity = e;
//                assert( entity->id() != 0 );
//                break;
//            }
//        }
//        if( entity == NULL )
//        {
//            return NULL;
//        }
//    }
//    return entity;
}
}
#endif
