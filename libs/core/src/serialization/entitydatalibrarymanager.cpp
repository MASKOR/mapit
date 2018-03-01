/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2016 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include <upns/serialization/entitydatalibrarymanager.h>
#include <sstream>
#include <algorithm>
#include <upns/logging.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace upns
{

std::shared_ptr<AbstractEntitydata> wrapEntityOfType(const std::string &type,
                                                                  std::shared_ptr<AbstractEntitydataProvider> streamProvider)
{
    std::string layertypeName(type);

    // Remove signs forbidden in filenames.
    char illegalCharacters[] = ".\\/<>$!#\0:\"|?*";
    for (unsigned int i = 0; i < strlen(illegalCharacters); ++i)
    {
       layertypeName.erase (std::remove(layertypeName.begin(), layertypeName.end(), illegalCharacters[i]), layertypeName.end());
    }
#ifndef NDEBUG
    std::string debug = DEBUG_POSTFIX;
#else
    std::string debug = "";
#endif

#ifdef _WIN32
    std::string prefix = "";
    std::string postfix = ".dll";
#else
    std::string prefix = "lib";
    std::string postfix = ".so";
#endif
    std::stringstream filenam;
    filenam << prefix << UPNS_INSTALL_LAYERTYPES << layertypeName << debug << postfix;
    std::stringstream fixpathfilename;
    fixpathfilename << "./libs/" << filenam.str();
#ifdef _WIN32
    HMODULE handle = LoadLibrary(fixpathfilename.str().c_str());
#else
    void* handle = dlopen(fixpathfilename.str().c_str(), RTLD_NOW);
#endif
#ifdef _WIN32
    DWORD prevDw = GetLastError();
#else
    char* prevDlErr = dlerror();
#endif
    if (!handle) {
        //std::stringstream systempathfilename;
        //systempathfilename << "upns_layertypes/" << filenam.str();
    #ifdef _WIN32
        handle = LoadLibrary(filenam.str().c_str());
    #else
        handle = dlopen(filenam.str().c_str(), RTLD_NOW);
    #endif
        if (!handle) {
        #ifdef _WIN32
            DWORD dw = GetLastError();
            std::cerr << "Cannot open library: " << fixpathfilename.str() << '\n'
                      << "errorcode: " << prevDw
                      << "nor: " << filenam.str() << '\n'
                      << "errorcode: " << dw << '\n';
        #else
            std::cerr << "Cannot open library: " << fixpathfilename.str() << '\n'
                      << prevDlErr << '\n'
                      << "nor: " << filenam.str() << '\n'
                      << dlerror() << '\n';
        #endif
            return std::shared_ptr<AbstractEntitydata>(NULL);
        }
    }
#ifdef _WIN32
    CreateEntitydataFunc wrap = (CreateEntitydataFunc)GetProcAddress(handle,"createEntitydata");
#else
    CreateEntitydataFunc wrap = (CreateEntitydataFunc)dlsym(handle, "createEntitydata");
#endif
    std::shared_ptr<AbstractEntitydata> ret;
    wrap(&ret, streamProvider);
    return ret;
    //return std::shared_ptr<AbstractEntitydata>( wrap( streamProvider ) );
}

//std::shared_ptr<AbstractEntitydata> wrapEntityOfType(LayerType type,
//                                                                   std::shared_ptr<AbstractEntitydataProvider> streamProvider)
//{
//    // Layertypes loosly coupled. Name is used to call library to handle concrete datatypes.
//    std::string layerName;
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
//        return std::shared_ptr<AbstractEntitydata>(NULL);
//    }
//    return wrapEntityOfType( layerName, streamProvider );
//}

std::shared_ptr<AbstractEntitydata> EntityDataLibraryManager::getEntitydataFromProvider(const std::string &type, std::shared_ptr<AbstractEntitydataProvider> edsp, bool canRead)
{
    std::shared_ptr<AbstractEntitydata> edata = wrapEntityOfType( type, edsp );
    return edata;
}

}
