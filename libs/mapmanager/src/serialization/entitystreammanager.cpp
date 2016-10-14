#include "serialization/entitystreammanager.h"
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
    std::stringstream filename;
    filename << "./libs/layertypes_collection/" << prefix << layertypeName << debug << postfix;
#ifdef _WIN32
    HMODULE handle = LoadLibrary(filename.str().c_str());
#else
    void* handle = dlopen(filename.str().c_str(), RTLD_NOW);
#endif
    if (!handle) {
#ifdef _WIN32
        DWORD dw = GetLastError();
        std::cerr << "Cannot open library: " << filename.str() <<  "errorcode: " << dw << '\n';
#else
        std::cerr << "Cannot open library: " << dlerror() << '\n';
#endif
        return upnsSharedPointer<AbstractEntityData>(NULL);
    }
#ifdef _WIN32
    CreateEntitydataFunc wrap = (CreateEntitydataFunc)GetProcAddress(handle,"createEntitydata");
#else
    CreateEntitydataFunc wrap = (CreateEntitydataFunc)dlsym(handle, "createEntitydata");
#endif
    upnsSharedPointer<AbstractEntityData> ret;
    wrap(&ret, streamProvider);
    return ret;
    //return upnsSharedPointer<AbstractEntityData>( wrap( streamProvider ) );
}

upnsSharedPointer<AbstractEntityData> wrapEntityOfType(LayerType type,
                                                                   upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
{
    // Layertypes loosly coupled. Name is used to call library to handle concrete datatypes.
    upnsString layerName;
    switch(type)
    {
    case POINTCLOUD2:
    {
        layerName = "layertype_pointcloud2";
        break;
    }
    case POSES:
    {
        layerName = "layertype_tf";
        break;
    }
    default:
        log_error("Unknown layertype: " + std::to_string(type));
        return upnsSharedPointer<AbstractEntityData>(NULL);
    }
    return wrapEntityOfType( layerName, streamProvider );
}

upnsSharedPointer<AbstractEntityData> EntityStreamManager::getEntityDataImpl(upns::upnsSharedPointer<AbstractMapSerializer> serializer, const ObjectId &entityId, bool canRead)
{
    upnsSharedPointer<Entity> ent = serializer->getEntity( entityId );
    if( ent == NULL )
    {
        log_error("Entity not found." + entityId);
        return NULL;
    }
    assert( ent );
    upnsSharedPointer<AbstractEntityData> edata = wrapEntityOfType( ent->type(), serializer->getStreamProvider(entityId, canRead/*, canWrite*/) ) ;
    return edata;
}

upnsSharedPointer<AbstractEntityData> EntityStreamManager::getEntityDataByPathImpl(upns::upnsSharedPointer<AbstractMapSerializer> serializer, const Path &path, bool canRead, bool canWrite)
{
    upnsSharedPointer<Entity> ent = serializer->getEntityTransient( path );
    if( ent == NULL )
    {
        log_error("Entity not found." + path);
        return NULL;
    }
    assert( ent );
    upnsSharedPointer<AbstractEntityData> edata = wrapEntityOfType( ent->type(), serializer->getStreamProviderTransient(path, canRead, canWrite) ) ;
    return edata;
}

upnsSharedPointer<AbstractEntityData> EntityStreamManager::getEntityDataFromStreamImpl(LayerType type, upnsSharedPointer<AbstractEntityDataStreamProvider> edsp, bool canRead)
{
    upnsSharedPointer<AbstractEntityData> edata = wrapEntityOfType( type, edsp );
    return edata;
}

}
