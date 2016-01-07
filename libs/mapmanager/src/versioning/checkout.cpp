#include "checkout.h"
#include "upns.h"
#include <dlfcn.h>
#include <string>
#include <algorithm>
#include <log4cplus/logger.h>
#include "module.h"
#include "operationenvironmentimpl.h"

namespace upns
{


Checkout::Checkout(AbstractMapSerializer *serializer, const CommitId commitOrCheckoutId)
    :CheckoutImpl(serializer, commitOrCheckoutId)
{
}

Checkout::~Checkout()
{
}

OperationResult Checkout::doOperation(const OperationDescription &desc)
{
    //TODO: This code my belong to a class which handles operation-modules. A "listOperations" might be needed outside of "checkout".
    OperationEnvironmentImpl env(desc);
    env.setMapManager( this );
    env.setMapService( this->m_innerService );
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
    filename << "./operator_modules/" << desc.operatorname() << "/" << prefix << desc.operatorname() << debug << postfix;
    if(desc.operatorversion())
    {
        filename << "." << desc.operatorversion();
    }
    void* handle = dlopen(filename.str().c_str(), RTLD_NOW);
    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return OperationResult(UPNS_STATUS_ERR_MODULE_OPERATOR_NOT_FOUND, OperationDescription());
    }
    GetModuleInfo getModInfo = (GetModuleInfo)dlsym(handle, "getModuleInfo");
    ModuleInfo* info = getModInfo();
    StatusCode result = info->operate( &env );
    if(!upnsIsOk(result))
    {
        std::stringstream strm;
        strm << "operator '" << desc.operatorname() << "' reported an error. (code:" << result << ")";
        log_error(strm.str());
    }
    return OperationResult(result, env.outputDescription());
}

upnsSharedPointer<AbstractEntityData> Checkout::getEntityData(const ObjectId &entityId)
{
    return CheckoutImpl::getEntityDataImpl(entityId, true);
}

}
