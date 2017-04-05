#ifndef OPERATORLIBRARYMANAGER_H
#define OPERATORLIBRARYMANAGER_H

#include <upns/typedefs.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/module.h>

// WARNING: Use this header only if you don't want to use core reimplement .
/**
 *  Note: This class should usually not be used directly. An implementation for
 *  this class is part of core. It can be used via the repository methods (e.g. "doOperation").
 *  An own implementation may be needed. This class provides a loading mechanism for operators.
 */

namespace upns {

/// This is _not_ a duplication of struct ModuleInfo
/// ModuleInfo: can be used while library is opened, contains pointer to memory, belonging to shred lib (char* + operate)
/// OperatorInfo: can be used after shared library was closed (e.g. for listing operators)
struct OperatorInfo
{
    const std::string         compiler;       //< use moduleCompiler()
    const std::string         compilerConfig; //< use moduleCompilerConfig()
    const std::string         date;           //< compilation date
    const std::string         time;           //< compilation date
    const std::string         moduleName;     //< unique name of the module
    const std::string         description;    //< short description
    const std::string         author;         //< author of module
    const int                 moduleVersion;  //< version
    const int                 apiVersion;     //< mapmanager api version
    const std::string         layerType;      //< LayerType enum

    OperatorInfo(const ModuleInfo &o)
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
    {}
};


class OperatorLibraryManager
{
public:
    static upns::OperationResult doOperation(const mapit::msgs::OperationDescription &desc, CheckoutRaw *checkout);

    static std::vector<OperatorInfo> listOperators();
};

}

#endif
