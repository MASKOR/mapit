#include <upns/operators/module.h>
#include <upns/serialization/entitydatalibrarymanager.h>
#include <upns/errorcodes.h>
#include "operationenvironmentimpl.h"
#include <sstream>
#include <algorithm>
#include <upns/logging.h>
#include <upns/serialization/operatorlibrarymanager.h>

#include <dirent.h>

#ifdef _WIN32
#include <windows.h>

// list files
#include <tchar.h>
#include <stdio.h>
#else
#include <dlfcn.h>
#endif

namespace upns
{

#ifdef _WIN32
typedef HMODULE HandleOpModule;
#else
typedef void* HandleOpModule;
#endif

std::string operatorLibraryName(const OperatorDescription &desc)
{
    std::stringstream filenam;
    filenam << prefix << UPNS_INSTALL_OPERATORS << desc.operatorname() << debug << postfix;
    if(desc.operatorversion())
    {
        filenam << "." << desc.operatorversion();
    }
}

std::pair<HandleOpModule, ModuleInfo*> loadOperatorModule(const OperatorDescription &desc)
{
#ifndef NDEBUG
    std::string debug(DEBUG_POSTFIX);
#else
    std::string debug("");
#endif

#ifdef _WIN32
    std::string prefix("");
    std::string postfix(".dll");
#else
    std::string prefix("lib");
    std::string postfix(".so");
#endif

    std::stringstream fixpathfilenam;
    fixpathfilenam << "./libs/operator_modules_collection/" << desc.operatorname() << "/" << filenam.str();
    std::string filenamestr = fixpathfilenam.str();
    log_info("loading operator module \"" + filenamestr + "\"");
    std::pair<HandleOpModule, ModuleInfo*> result;
#ifdef _WIN32
    HMODULE handle = LoadLibrary(fixpathfilenam.str().c_str());
#else
    void* handle = dlopen(fixpathfilenam.str().c_str(), RTLD_NOW);
#endif
    if (!handle) {
        std::stringstream systempathfilenam;
        systempathfilenam << filenam.str();
        filenamestr = systempathfilenam.str();
        log_info("loading operator module \"" + filenamestr + "\"");
    #ifdef _WIN32
        handle = LoadLibrary(filenamestr.c_str());
    #else
        handle = dlopen(filenamestr.c_str(), RTLD_NOW);
    #endif
        if (!handle) {
        #ifdef _WIN32
        #else
            std::cerr << "Cannot open library: " << dlerror() << '\n';
        #endif
            result.first = nullptr;
            return result;
            //return OperationResult(UPNS_STATUS_ERR_MODULE_OPERATOR_NOT_FOUND, OperationDescription());
        }
    }
    result.first = handle;
#ifdef _WIN32
    //FARPROC getModInfo = GetProcAddress(handle, "getModuleInfo");
    GetModuleInfo getModInfo = (GetModuleInfo)GetProcAddress(handle, "getModuleInfo");
#else
    GetModuleInfo getModInfo = (GetModuleInfo)dlsym(handle, "getModuleInfo");
#endif
    result.second = getModInfo();
    return result;
}

OperationResult _doOperation(const ModuleInfo *module, const upns::OperationDescription &desc, CheckoutRaw *checkout)
{
//    OperationDescription opdesc;
//    // protobuf has no methods to handle members as const. We won't change opdesc.operator()
//    opdesc.set_allocated_operator_(const_cast<OperatorDescription*>(&desc));
//    opdesc.set_params(params);
    OperationEnvironmentImpl env(desc);
    env.setCheckout( checkout );
    StatusCode result = module->operate( &env );
    if(!upnsIsOk(result))
    {
        std::stringstream strm;
        strm << "operator '" << desc.operator_().operatorname() << "' reported an error. (code:" << result << ")";
        log_error(strm.str());
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
        result = UPNS_STATUS_OK;
    } else {
        //TODO: GetLastError()...
        result = UPNS_STATUS_ERR_UNKNOWN;
    }
#else
    int status = dlclose(handle);
    if(status == 0) {
        result = UPNS_STATUS_OK;
    } else {
        result = UPNS_STATUS_ERR_UNKNOWN;
    }
#endif
    return result;
}

OperationResult OperatorLibraryManager::doOperation(const OperationDescription &desc, CheckoutRaw *checkout)
{
    std::pair<HandleOpModule, ModuleInfo*> modInfo = loadOperatorModule(desc.operator_());
    if(!modInfo.first)
    {
        return OperationResult(UPNS_STATUS_ERR_MODULE_OPERATOR_NOT_FOUND, OperationDescription());
    }
    OperationResult result = _doOperation(modInfo.second, desc, checkout);
    if(!upnsIsOk(result.first))
    {
        std::stringstream strm;
        strm << "operator '" << desc.operator_().operatorname() << "' reported an error. (code:" << result.first << ")";
        log_error(strm.str());
    }
    StatusCode status = closeOperatorModule(modInfo.first);
    if(!upnsIsOk(status))
    {
        std::stringstream strm;
        strm << "operator '" << desc.operator_().operatorname() << "' could not be unloaded. (code:" << status << ")";
        log_error(strm.str());
    }
    //Note: returnvalue is always from doOperation, if execution came thus far.
    return result;
}

void addOperatorsFromDirectory(std::vector<ModuleInfo> &vec, std::string& dirname)
{
    std::string prefix("lib"UPNS_INSTALL_OPERATORS);
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
    if ((dir = opendir (dirname.c_str())) != nullptr) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != nullptr) {
            std::string name(ent->d_name);
            if(!name.compare(0, prefix.length(), prefix))
            {
                loadOperatorModule()
            }
        }
        closedir (dir);
    } else {
        /* could not open directory */
        perror ("");
        return EXIT_FAILURE;
    }
#endif
}

std::vector<ModuleInfo> OperatorLibraryManager::listOperators()
{

}



}
