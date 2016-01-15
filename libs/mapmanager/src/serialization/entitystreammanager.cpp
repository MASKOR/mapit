#include "entitystreammanager.h"
#include "dlfcn.h"

namespace upns
{

upnsSharedPointer<AbstractEntityData> wrapEntityOfType(upnsString layertypeName,
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

upnsSharedPointer<AbstractEntityData> wrapEntityOfType(LayerType type,
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

upnsSharedPointer<AbstractEntityData> EntityStreamManager::getEntityDataImpl(AbstractMapSerializer* serializer, const ObjectId &entityId, bool canRead, bool canWrite)
{
    upnsSharedPointer<Entity> ent = serializer->getEntity( entityId );
    if( ent == NULL )
    {
        log_error("Entity not found." + entityId);
        return NULL;
    }
    assert( ent );
    upnsSharedPointer<AbstractEntityData> edata = wrapEntityOfType( ent->type(), serializer->getStreamProvider(entityId, canRead, canWrite) ) ;
    return edata;
}

}
