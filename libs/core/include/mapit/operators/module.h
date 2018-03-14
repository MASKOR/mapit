/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef __MODULE_H
#define __MODULE_H


#include <mapit/entitydata.h>
//#include <boost/config.hpp>
#include <mapit/msgs/services.pb.h>

// Always export. Headernot needed for import, because of dynamic loading at runtime.
#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif

#define MAPIT_MODULE_API_VERSION 0001

/**
 * Every module can include this header and implement the functions. Implementations of the operator module are contained in a binary file.
 *
 * Note to binary compatibility: Modules do heavy computations on data and should be as fast as possible. Thus the original data of the layer is
 * given to the module without modifications. This is a c++-structure in contrast to POD (plain old data) and thus will not be binary compatible
 * unless compiler, compiler version, os, third party dependencies are identical between module and mapamanger.
 * It could be tried to overcome this issue by providing an C interface to the \sa LayerData. However, datastrutures are likely to be c++ / incompatible.
 *
 * For fast Operation: Operator receives the original data and can decide to alter the exiting data or return a newly allocated chunk.
 * Behaviour can be controlled by the application (use caching). E.g. a transformation can occur on the data itself. Only the TF must be changed, Pointclouddata remains and does not get copied.
 * For a voxelgridfilter, both may be stored: the original pointcloud and the filtered one. The new version of the layer can be used, and the old one is also "cached" until the system decides,
 * it is no longer used.
 */

extern "C"
{
namespace mapit {
class OperationEnvironment;
}

typedef mapit::StatusCode (*OperateFunc)(mapit::OperationEnvironment*);

struct ModuleInfo
{
    const char*         compiler;       //< use moduleCompiler()
    const char*         compilerConfig; //< use moduleCompilerConfig()
    const char*         date;           //< compilation date
    const char*         time;           //< compilation date
    const char*         moduleName;     //< unique name of the module
    const char*         description;    //< short description
    const char*         author;         //< author of module
    const int           moduleVersion;  //< version
    const int           apiVersion;     //< mapit api version
    const char*         layerType;      //< LayerType enum
    const OperateFunc   operate;
    ModuleInfo(const ModuleInfo &o)
        : compiler(o.compiler)
        , compilerConfig(o.compilerConfig)
        , date(o.date)
        , time(o.time)
        , moduleName(o.moduleName)
        , description(o.description)
        , author(o.author)
        , moduleVersion(o.moduleVersion)
        , apiVersion(o.apiVersion)
        , layerType(o.layerType)
        , operate(o.operate)
    {}
    ModuleInfo(  const char*       compiler
               , const char*       compilerConfig
               , const char*       date
               , const char*       time
               , const char*       moduleName
               , const char*       description
               , const char*       author
               , const int         moduleVersion
               , const int         apiVersion
               , const char*       layerType
               , const OperateFunc operate)
    : compiler(compiler)
    , compilerConfig(compilerConfig)
    , date(date)
    , time(time)
    , moduleName(moduleName)
    , description(description)
    , author(author)
    , moduleVersion(moduleVersion)
    , apiVersion(apiVersion)
    , layerType(layerType)
    , operate(operate)
{}

private:
    // Prevent manual deletion of returned pointer
    ~ModuleInfo() {}
    friend MODULE_EXPORT ModuleInfo* getModuleInfo();
};
}

#ifdef __GNUC__
static const char* g_compiler = "GNU";
#elif _MSC_VER
static const char* g_compiler = "MSVC";
//TODO: Add more compilers if needed...
#endif
#ifdef __GNUC__
static const char* g_compilerconfig = "TODO";
#elif _MSC_VER
static const char* g_compilerconfig = "TODO";
#endif

typedef ModuleInfo* (*GetModuleInfo)();

#define MAPIT_MODULE(moduleName, description, author, moduleVersion, layerType, operateFunc) \
  extern "C" { \
      MODULE_EXPORT ModuleInfo* getModuleInfo() \
      { \
          static ModuleInfo info = ModuleInfo( g_compiler \
                                             , g_compilerconfig \
                                             , __DATE__ \
                                             , __TIME__ \
                                             , moduleName \
                                             , description \
                                             , author \
                                             , moduleVersion \
                                             , MAPIT_MODULE_API_VERSION \
                                             , layerType \
                                             , operateFunc ); \
          return &info; \
      } \
  }

#endif
