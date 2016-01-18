#include "checkoutimpl.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#include <string>
#include <sstream>
#include <algorithm>
#include <log4cplus/logger.h>
#include "module.h"
#include "operationenvironmentimpl.h"
#include "serialization/entitystreammanager.h"

#include <QDir>

namespace upns
{
typedef ModuleInfo* (*GetModuleInfo)();

CheckoutImpl::CheckoutImpl(AbstractMapSerializer *serializer, upnsSharedPointer<CheckoutObj>  checkoutCommit, const upnsString &branchname)
    :m_serializer(serializer),
     m_branchname( branchname ),
     m_checkout(checkoutCommit)
{
//    if(m_serializer->isCheckout(checkoutCommit))
//    {
//        m_checkoutId = commitOrCheckoutId;
//    }
//    else if(m_serializer->isCommit(commitOrCheckoutId))
//    {
//        upnsSharedPointer<Commit> co(new Commit());
//        co->add_parentcommitids(commitOrCheckoutId);
//        m_checkoutId = m_serializer->createCheckoutCommit( co );
//    }
}

//CheckoutImpl::CheckoutImpl(AbstractMapSerializer *serializer, const upnsSharedPointer<Branch> &branch)
//    :m_serializer(serializer),
//      m_branch( branch )
//{
////    if(m_serializer->isCheckout(branch->commitid()))
////    {
////        m_checkoutId = branch->commitid();
////    }
////    else
////    {
////        upnsSharedPointer<Commit> co(new Commit());
////        if(m_serializer->isCommit(branch->commitid()))
////        {
////            co->add_parentcommitids(branch->commitid());
////        }
////        else
////        {
////            log_info("Initial Commit created, branch: " + branch->name());
////        }
////        m_checkoutId = m_serializer->createCheckoutCommit( co );
////    }
//}

CheckoutImpl::~CheckoutImpl()
{
}

bool CheckoutImpl::isInConflictMode()
{
    return false;
}

upnsVec<upnsSharedPointer<Conflict> > CheckoutImpl::getPendingConflicts()
{
    return upnsVec<upnsSharedPointer<Conflict> >();
}

upnsSharedPointer<Tree> CheckoutImpl::getRoot()
{
    return NULL;
}

upnsSharedPointer<Tree> CheckoutImpl::getTree(const Path &path)
{
    return NULL;
}

upnsSharedPointer<Entity> CheckoutImpl::getEntity(const Path &path)
{
    return NULL;
}

upnsSharedPointer<Tree> CheckoutImpl::getTreeConflict(const ObjectId &objectId)
{
    return NULL;
}

upnsSharedPointer<Entity> CheckoutImpl::getEntityConflict(const ObjectId &objectId)
{
    return NULL;
}

OperationResult CheckoutImpl::doOperation(const OperationDescription &desc)
{
    //TODO: This code my belong to a class which handles operation-modules. A "listOperations" might be needed outside of "checkout".
    OperationEnvironmentImpl env(desc);
    env.setCheckout( this );
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
    filename << "./libs/operator_modules_collection/" << desc.operatorname() << "/" << prefix << desc.operatorname() << debug << postfix;
    if(desc.operatorversion())
    {
        filename << "." << desc.operatorversion();
    }
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
        return OperationResult(UPNS_STATUS_ERR_MODULE_OPERATOR_NOT_FOUND, OperationDescription());
    }
#ifdef _WIN32
    //FARPROC getModInfo = GetProcAddress(handle,"getModuleInfo");
    GetModuleInfo getModInfo = (GetModuleInfo)GetProcAddress(handle,"getModuleInfo");
#else
    GetModuleInfo getModInfo = (GetModuleInfo)dlsym(handle, "getModuleInfo");
#endif
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

upnsSharedPointer<AbstractEntityData> CheckoutImpl::getEntityDataReadOnly(const Path &path)
{
    ObjectId oid;
    return EntityStreamManager::getEntityDataImpl(m_serializer, oid, true, false);
}

upnsSharedPointer<AbstractEntityData> CheckoutImpl::getEntityDataReadOnlyConflict(const ObjectId &entityId)
{
    return EntityStreamManager::getEntityDataImpl(m_serializer, entityId, false, true);
}

upnsSharedPointer<AbstractEntityData> CheckoutImpl::getEntityDataForReadWrite(const Path &path)
{
    ObjectId oid;
    return EntityStreamManager::getEntityDataImpl(m_serializer, oid, true, true);
}

StatusCode CheckoutImpl::storeTree(const Path &path, upnsSharedPointer<Tree> tree)
{
    //return m_serializer->storeTree();
    return 0;
}

StatusCode CheckoutImpl::storeEntity(const Path &path, upnsSharedPointer<Entity> tree)
{
    return 0;
}

void CheckoutImpl::setConflictSolved(const Path &path, const ObjectId &oid)
{

}

ObjectId CheckoutImpl::oidForChild(upnsSharedPointer<Tree> tree, const ::std::string &name)
{
    const ::google::protobuf::Map< ::std::string, ::upns::ObjectReference > &refs = tree->refs();
    ::google::protobuf::Map< ::std::string, ::upns::ObjectReference >::const_iterator iter(refs.cbegin());
    while(iter != refs.cend())
    {
        if(iter->first == name) return iter->second.id();
        iter++;
    }
    return "";
}

ObjectId CheckoutImpl::oidForPath(const Path &path)
{
//    m_checkout->commit().root;
    upnsSharedPointer<Tree> current(getRoot());
    Path p;
    if(path[0] == '/')
    {
        p = path.substr(1);
    }
    else
    {
        p = path;
    }
    while(true)
    {
        size_t nextSlash = p.find_first_of('/');
        assert(nextSlash != 0);
        if(nextSlash == std::string::npos)
        {
            return "";
        }

    }
}

}
