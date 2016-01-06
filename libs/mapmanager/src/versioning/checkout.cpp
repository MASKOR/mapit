#include "checkout.h"

namespace upns
{


upnsSharedPointer<AbstractEntityData> Checkout::getEntityData(const ObjectId &entityId)
{
    upnsVec<MapIdentifier> mapIds;
    mapIds.push_back( mapId );
    upnsSharedPointer<Entity> ent = m_serializer->getEntity( entityId );
    if( ent == NULL )
    {
        log_error("Entity not found." + entityId);
        return NULL;
    }
    const Layer *layer = NULL;
    for(int i=0 ; i<map->layers_size() ; ++i)
    {
        const Layer &curLayer = map->layers(i);
        if(curLayer.id() == layerId)
        {
            layer = &curLayer;
            break;
        }
    }
    assert( layer );

    upnsSharedPointer<AbstractEntityData> layerData = wrapEntityOfType( ent->type(), m_serializer->getStreamProvider(entityId) ) ;
    return layerData;
}

upnsSharedPointer<AbstractEntityData> Checkout::wrapEntityOfType(LayerType type,
                                                                   upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
{
    upnsString layerName;
    switch(type)
    {
    case POINTCLOUD2:
    {
        layerName = "layertype_pointcloud2";
        break;
    }
    default:
        log_error("Unknown layertype: " + std::to_string(type));
        return upnsSharedPointer<AbstractEntityData>(NULL);
    }
    return wrapEntityOfType( layerName, streamProvider );
}

upnsSharedPointer<AbstractEntityData> Checkout::wrapEntityOfType(upnsString layertypeName,
                                                                  upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
{
#ifndef NDEBUG
    upnsString debug = DEBUG_POSTFIX;
#else
    upnsString debug = "";
#endif

#ifdef _WIN32
    upnsString prefix = "";
    upnsString postfix = ".dll";
#else
    upnsString prefix = "lib";
    upnsString postfix = ".so";
#endif
    void* handle = dlopen((upnsString("./layertypes/") + prefix + layertypeName + debug + postfix).c_str(), RTLD_NOW);
    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return upnsSharedPointer<AbstractEntityData>(NULL);
    }
    WrapLayerTypeFunc wrap = (WrapLayerTypeFunc)dlsym(handle, "createEntityData");
    return upnsSharedPointer<AbstractEntityData>( wrap( streamProvider ) );
}

}
