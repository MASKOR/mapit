#include "serialization/entitystreammanager.h"
#include <sstream>
#include <algorithm>
#include "upns_logging.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace upns
{

upnsSharedPointer<AbstractEntitydata> wrapEntityOfType(const std::string &type,
                                                                  upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider)
{
    std::string layertypeName(type);

    // Remove signs forbidden in filenames.
    char illegalCharacters[] = ".\\/<>$!#\0:\"|?*";
    for (unsigned int i = 0; i < strlen(illegalCharacters); ++i)
    {
       layertypeName.erase (std::remove(layertypeName.begin(), layertypeName.end(), illegalCharacters[i]), layertypeName.end());
    }
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
        return upnsSharedPointer<AbstractEntitydata>(NULL);
    }
#ifdef _WIN32
    CreateEntitydataFunc wrap = (CreateEntitydataFunc)GetProcAddress(handle,"createEntitydata");
#else
    CreateEntitydataFunc wrap = (CreateEntitydataFunc)dlsym(handle, "createEntitydata");
#endif
    upnsSharedPointer<AbstractEntitydata> ret;
    wrap(&ret, streamProvider);
    return ret;
    //return upnsSharedPointer<AbstractEntitydata>( wrap( streamProvider ) );
}

//upnsSharedPointer<AbstractEntitydata> wrapEntityOfType(LayerType type,
//                                                                   upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider)
//{
//    // Layertypes loosly coupled. Name is used to call library to handle concrete datatypes.
//    upnsString layerName;
//    switch(type)
//    {
//    case POINTCLOUD:
//    {
//        layerName = "layertype_pointcloud2";
//        break;
//    }
//    case TF:
//    {
//        layerName = "layertype_tf";
//        break;
//    }
//    case OPENVDB:
//    {
//        layerName = "layertype_openvdb";
//        break;
//    }
//    case ASSET:
//    {
//        layerName = "layertype_asset";
//        break;
//    }
//    default:
//        log_error("Unknown layertype: " + std::to_string(type));
//        return upnsSharedPointer<AbstractEntitydata>(NULL);
//    }
//    return wrapEntityOfType( layerName, streamProvider );
//}

upnsSharedPointer<AbstractEntitydata> EntityStreamManager::getEntitydataFromStreamImpl(const std::string &type, upnsSharedPointer<AbstractEntitydataStreamProvider> edsp, bool canRead)
{
    upnsSharedPointer<AbstractEntitydata> edata = wrapEntityOfType( type, edsp );
    return edata;
}

}
