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


class OperatorLibraryManager
{
public:
    static upns::OperationResult doOperation(const mapit::msgs::OperationDescription &desc, CheckoutRaw *checkout);

    static std::vector<ModuleInfo> listOperators();
};

}

#endif
