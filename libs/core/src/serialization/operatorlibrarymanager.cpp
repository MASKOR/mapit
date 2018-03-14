/*******************************************************************************
 *
 * Copyright 2017-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include <mapit/operators/module.h>
#include <mapit/serialization/entitydatalibrarymanager.h>
#include <mapit/errorcodes.h>
#include "operationenvironmentimpl.h"
#include <sstream>
#include <algorithm>
#include <mapit/logging.h>
#include <mapit/serialization/operatorlibrarymanager.h>

#include <dirent.h>

#ifdef _WIN32
#include <windows.h>

// list files
#include <tchar.h>
#include <stdio.h>
#else
#include <dlfcn.h>
#endif

#define MAPIT_LOCAL_OPERATOR_DIR "./libs/"

namespace mapit
{

#ifdef _WIN32
typedef HMODULE HandleOpModule;
#else
typedef void* HandleOpModule;
#endif

std::string operatorLibraryName(const mapit::msgs::OperatorDescription &desc)
{
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
    filenam << prefix << MAPIT_INSTALL_OPERATORS << desc.operatorname() << debug << postfix;
    if(desc.operatorversion())
    {
        filenam << "." << desc.operatorversion();
    }
    return filenam.str();
}

HandleOpModule loadOperatorModule(const std::string filename, bool printerror = true)
{
#ifdef _WIN32
    HandleOpModule handle = LoadLibrary(filename.c_str());
#else
    HandleOpModule handle = dlopen(filename.c_str(), RTLD_NOW);
#endif
    if (!handle && printerror) {
    #ifdef _WIN32
        log_warn("Cannot open library: " + filename);
    #else
        log_warn("Cannot open library: " + dlerror());
    #endif
    }
    return handle;
}

HandleOpModule loadOperatorModule(const mapit::msgs::OperatorDescription &desc)
{
    std::string filenam = operatorLibraryName(desc);
    std::stringstream fixpathfilenam;
    fixpathfilenam << MAPIT_LOCAL_OPERATOR_DIR << filenam;
    std::string fixedfilenamestr = fixpathfilenam.str();
    log_info("loading operator module \"" + fixedfilenamestr + "\"");
    std::pair<HandleOpModule, ModuleInfo*> result;
    HandleOpModule handle = loadOperatorModule(fixedfilenamestr, false);
    if (!handle) {
        std::string prevdlerror( dlerror() );
        std::string systemfilenamestr = filenam;
        log_info("loading operator module \"" + systemfilenamestr + "\"");
        handle = loadOperatorModule(systemfilenamestr, false);
        if (!handle) {
            log_warn("Cannot open library: " + prevdlerror + " or " + dlerror());
        }
    }
    return handle;
}

ModuleInfo* getModuleInfo(HandleOpModule &handle)
{
#ifdef _WIN32
    //FARPROC getModInfo = GetProcAddress(handle, "getModuleInfo");
    GetModuleInfo getModInfo = (GetModuleInfo)GetProcAddress(handle, "getModuleInfo");
#else
    GetModuleInfo getModInfo = (GetModuleInfo)dlsym(handle, "getModuleInfo");
#endif
    if(!getModInfo)
    {
        log_error("could not get symbol \"getModuleInfo\".");
        return nullptr;
    }
    return getModInfo();
}

mapit::OperationResult _doOperation(const ModuleInfo *module, const mapit::msgs::OperationDescription &desc, CheckoutRaw *checkout)
{
//    OperationDescription opdesc;
//    // protobuf has no methods to handle members as const. We won't change opdesc.operator()
//    opdesc.set_allocated_operator_(const_cast<OperatorDescription*>(&desc));
//    opdesc.set_params(params);
    OperationEnvironmentImpl env(desc);
    env.setCheckout( checkout );
    StatusCode result = module->operate( &env );
    if(!mapitIsOk(result))
    {
        std::stringstream strm;
        strm << "operator '" << desc.operator_().operatorname() << "' reported an error. (code:" << result << ")";
        log_warn(strm.str());
    }
//    opdesc.release_operator_();
    return OperationResult(result, env.outputDescription());
}

StatusCode closeOperatorModule(HandleOpModule handle)
{
    assert(handle);
    StatusCode result;
#ifdef _WIN32
    bool success = FreeLibrary(handle);
    if(success) {
        result = MAPIT_STATUS_OK;
    } else {
        //TODO: GetLastError()...
        result = MAPIT_STATUS_ERR_UNKNOWN;
        log_warn("could not close library");
    }
#else
    int status = dlclose(handle);
    if(status == 0) {
        result = MAPIT_STATUS_OK;
    } else {
        result = MAPIT_STATUS_ERR_UNKNOWN;
        log_warn("could not close library");
    }
#endif
    return result;
}

OperationResult OperatorLibraryManager::doOperation(const mapit::msgs::OperationDescription &desc, CheckoutRaw *checkout)
{
    HandleOpModule handle = loadOperatorModule(desc.operator_());
    if(!handle)
    {
        return OperationResult(MAPIT_STATUS_ERR_MODULE_OPERATOR_NOT_FOUND, mapit::msgs::OperationDescription());
    }
    log_info("open library: " + desc.operator_().operatorname());
    ModuleInfo* modInfo = getModuleInfo(handle);
    if(!modInfo)
    {
        return OperationResult(MAPIT_STATUS_ERR_MODULE_OPERATOR_NOT_FOUND, mapit::msgs::OperationDescription());
    }
    OperationResult result = _doOperation(modInfo, desc, checkout);
    if(!mapitIsOk(result.first))
    {
        std::stringstream strm;
        strm << "operator '" << desc.operator_().operatorname() << "' reported an error. (code:" << result.first << ")";
        log_warn(strm.str());
    }
    StatusCode status = closeOperatorModule(handle);
    if(!mapitIsOk(status))
    {
        std::stringstream strm;
        strm << "operator '" << desc.operator_().operatorname() << "' could not be unloaded. (code:" << status << ")";
        log_error(strm.str());
    }
    //Note: returnvalue is always from doOperation, if execution came thus far.
    return result;
}

void addOperatorsFromDirectory(std::vector<OperatorInfo> &vec, const std::string dirname, bool recursive = false)
{
    std::string prefix("lib" MAPIT_INSTALL_OPERATORS);

    std::string path = dirname;
    if(path.length() != 0 && path[path.length()-1] != '/')
    {
        path += "/";
    }

#ifdef _WIN32
    //TODO
    WIN32_FIND_DATA FindFileData;
    HANDLE hFind;
    hFind = FindFirstFile(argv[1], &FindFileData);
    if (hFind == INVALID_HANDLE_VALUE)
    {
        log_error("FindFirstFile failed" + GetLastError());
        return;
    }
    else
    {
        FindFileData.cFileName
        FindClose(hFind);
    }
#else
    DIR *dir;
    dirent *ent;
    if ((dir = opendir (dirname.c_str())) == nullptr)
    {
        log_warn("could not open directory" << dirname);
        return;
    }
    /* print all the files and directories within directory */
    while ((ent = readdir (dir)) != nullptr) {
        std::string name(ent->d_name);
        if(ent->d_type == DT_DIR)
        {
            if(   recursive
               && strcmp(ent->d_name, ".") != 0
               && strcmp(ent->d_name, "..") != 0)
            {
                addOperatorsFromDirectory(vec, path + name, recursive);
            }
        }
        else if(!name.compare(0, prefix.length(), prefix))
        {
            HandleOpModule handle = loadOperatorModule(path + name, true);
            if(!handle)
            {
                //log_warn("could not open library: " + name);
                continue;
            }
            log_info("open library: " + name);
            ModuleInfo* modInfo = getModuleInfo(handle);
            if(!modInfo)
            {
                log_warn("not a library: " + name);
                continue;
            }

            //TODO: different version may be listed!
            bool duplicate=false;
            std::string name(modInfo->moduleName);
            for(std::vector<OperatorInfo>::const_iterator iter(vec.cbegin()) ; iter != vec.cend() ; iter++)
            {
                if(name.compare(std::string(iter->moduleName)) == 0)
                {
                    duplicate = true;
                    break;
                }
            }
            vec.push_back(OperatorInfo(*modInfo));
            StatusCode status = closeOperatorModule(handle);
            if(!mapitIsOk(status))
            {
                log_warn("Could not close library: " + name);
            }
        }
    }
    closedir (dir);
#endif
}

std::vector<OperatorInfo> OperatorLibraryManager::listOperators()
{
    std::vector<OperatorInfo> moduleInfos;
    log_info("loading operators from: " + MAPIT_LOCAL_OPERATOR_DIR);
    addOperatorsFromDirectory(moduleInfos, MAPIT_LOCAL_OPERATOR_DIR, true);
    log_info("loading operators from: /usr/lib/");
    addOperatorsFromDirectory(moduleInfos, "/usr/lib/");
    log_info("loading operators from: /usr/local/lib/");
    addOperatorsFromDirectory(moduleInfos, "/usr/local/lib/");
    return moduleInfos;
}



}
