#include "entitystreammanager.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

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
    std::stringstream filename("./layertypes/");
    filename << prefix << layertypeName << debug << postfix;
#ifdef _WIN32
    HMODULE handle = LoadLibrary(filename.str().c_str());
#else
    void* handle = dlopen(filename.str().c_str(), RTLD_NOW);
#endif
    if (!handle) {
#ifdef _WIN32
#else
        std::cerr << "Cannot open library: " << dlerror() << '\n';
#endif
        return upnsSharedPointer<AbstractEntityData>(NULL);
    }
#ifdef _WIN32
    //FARPROC getModInfo = GetProcAddress(handle,"getModuleInfo");
    WrapLayerTypeFunc wrap = (WrapLayerTypeFunc)GetProcAddress(handle,"createEntityData");
#else
    WrapLayerTypeFunc wrap = (WrapLayerTypeFunc)dlsym(handle, "createEntityData");
#endif
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
