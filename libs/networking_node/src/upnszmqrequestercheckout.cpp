#include "upnszmqrequestercheckout.h"
#include <upns/services_internal.pb.h>
#include <upns/serialization/entitydatalibrarymanager.h> //TODO: put in yet another independent project/cmake target. No dependecy to mapmanager required.
#include "serialization/zmqentitydatastreamprovider.h"
#include "operationenvironmentimpl.h"
#include <upns/errorcodes.h>
#include <upns/depthfirstsearch.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#include <upns/operators/module.h>
typedef ModuleInfo* (*GetModuleInfo)();

upns::ZmqRequesterCheckout::ZmqRequesterCheckout(std::string name, ZmqProtobufNode *node, Checkout *cache, bool operationsLocal)
    :m_checkoutName( name ),
     m_node( node ),
     m_cache( cache ),
     m_operationsLocal( operationsLocal )
{
}

bool upns::ZmqRequesterCheckout::isInConflictMode()
{
    //TODO: nyi
    assert(false);
    return false;
}

std::vector<std::shared_ptr<upns::Conflict> > upns::ZmqRequesterCheckout::getPendingConflicts()
{
    //TODO: nyi
    assert(false);
    return std::vector<std::shared_ptr<upns::Conflict> >();
}

void upns::ZmqRequesterCheckout::setConflictSolved(const upns::Path &path, const upns::ObjectId &oid)
{
    //TODO: nyi
    assert(false);
}

upns::MessageType upns::ZmqRequesterCheckout::typeOfObject(const upns::Path &oidOrName)
{
    //TODO: Introduce typeof method with protobuf
    if(this->getTree(oidOrName) != nullptr) return MessageTree;
    if(this->getEntity(oidOrName) != nullptr) return MessageEntity;
    //if(this->get(oidOrName) != nullptr) return MessageCommit;
    //if(this->getTree(oidOrName) != nullptr) return MessageCheckout;
    //if(this->get(oidOrName) != nullptr) return MessageBranch;
    if(this->getEntitydataReadOnly(oidOrName) != nullptr) return MessageEntitydata;
    return MessageEmpty;
}

std::shared_ptr<upns::Tree> upns::ZmqRequesterCheckout::getRoot()
{
//    std::unique_ptr<upns::RequestHierarchy> req(new upns::RequestHierarchy);
//    req->set_checkout(m_checkoutName);
//    m_node->send(std::move(req));
//    std::unique_ptr<upns::ReplyHierarchy> rep(m_node->receive<upns::ReplyHierarchy>());
//    std::shared_ptr<upns::Tree> ret(new upns::Tree);
//    for(google::protobuf::Map< ::std::string, ::upns::ReplyHierarchyMap >::const_iterator ch( rep->maps().cbegin() );
//        ch != rep->maps().cend();
//        ++ch)
//    {
//        upns::ObjectReference ref;
//        ref.set_path(m_checkoutName + "/" + ch->first);
//        ret->mutable_refs()->insert(::google::protobuf::MapPair< ::std::string, ::upns::ObjectReference>(ch->first, upns::ObjectReference()));
//    }
//    return ret;
    return getTree("/");
}

std::shared_ptr<upns::Tree> upns::ZmqRequesterCheckout::getTreeConflict(const upns::ObjectId &objectId)
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<upns::Tree>();
}

std::shared_ptr<upns::Entity> upns::ZmqRequesterCheckout::getEntityConflict(const upns::ObjectId &objectId)
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<upns::Entity>();
}

std::shared_ptr<upns::Tree> upns::ZmqRequesterCheckout::getTree(const upns::Path &path)
{
    std::unique_ptr<upns::RequestGenericEntry> req(new upns::RequestGenericEntry);
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    m_node->send(std::move(req));
    std::shared_ptr<upns::ReplyGenericEntry> rep(m_node->receive<upns::ReplyGenericEntry>());
    std::shared_ptr<upns::Tree> ret(rep->mutable_entry()->release_tree());
    return ret;
}

std::shared_ptr<upns::Entity> upns::ZmqRequesterCheckout::getEntity(const upns::Path &path)
{
    std::unique_ptr<upns::RequestGenericEntry> req(new upns::RequestGenericEntry);
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    m_node->send(std::move(req));
    std::shared_ptr<upns::ReplyGenericEntry> rep(m_node->receive<upns::ReplyGenericEntry>());
    std::shared_ptr<upns::Entity> ret(rep->mutable_entry()->release_entity());
    return ret;
}

std::shared_ptr<upns::Branch> upns::ZmqRequesterCheckout::getParentBranch()
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<upns::Branch>();
}

std::vector<upns::CommitId> upns::ZmqRequesterCheckout::getParentCommitIds()
{
    //TODO: nyi
    assert(false);
    return std::vector<upns::CommitId>();
}

std::shared_ptr<upns::AbstractEntitydata> upns::ZmqRequesterCheckout::getEntitydataReadOnly(const upns::Path &entityId)
{
    std::shared_ptr<Entity> e = getEntity(entityId);
    if(!e)
    {
        log_error("Entity could not be queried for entitydata: " + entityId);
        return std::shared_ptr<upns::AbstractEntitydata>(nullptr);
    }
    std::shared_ptr<AbstractEntitydataProvider> streamProvider(new ZmqEntitydataStreamProvider(m_checkoutName, entityId, m_node));
    return EntityDataLibraryManager::getEntitydataFromProvider(e->type(), streamProvider);
}

std::shared_ptr<upns::AbstractEntitydata> upns::ZmqRequesterCheckout::getEntitydataReadOnlyConflict(const upns::ObjectId &entityId)
{
    //TODO: nyi
    assert(false);
    return std::shared_ptr<upns::AbstractEntitydata>(nullptr);
}

upns::StatusCode upns::ZmqRequesterCheckout::depthFirstSearch(std::function<bool (std::shared_ptr<upns::Commit>, const upns::ObjectReference &, const upns::Path &)> beforeCommit,
                                                              std::function<bool (std::shared_ptr<upns::Commit>, const upns::ObjectReference &, const upns::Path &)> afterCommit,
                                                              std::function<bool (std::shared_ptr<upns::Tree>, const upns::ObjectReference &, const upns::Path &)> beforeTree,
                                                              std::function<bool (std::shared_ptr<upns::Tree>, const upns::ObjectReference &, const upns::Path &)> afterTree,
                                                              std::function<bool (std::shared_ptr<upns::Entity>, const upns::ObjectReference &, const upns::Path &)> beforeEntity,
                                                              std::function<bool (std::shared_ptr<upns::Entity>, const upns::ObjectReference &, const upns::Path &)> afterEntity)
{
    //TODO: remove "Commit" from depthFirstSearch! There is no commit in checkout by design. (only technically)
    // Use internal service instead of ReplyHierarchy
    ObjectReference nullRef;
    StatusCode s = upns::depthFirstSearch(this, getRoot(), nullRef, "", beforeCommit, afterCommit, beforeTree, afterTree, beforeEntity, afterEntity);
    return s;
}

upns::OperationResult upns::ZmqRequesterCheckout::doOperation(const upns::OperationDescription &desc)
{
    // TODO: this breaks req/resp pattern!
    if(m_operationsLocal)
    {
        //TODO: This code my belong to a class which handles operation-modules. A "listOperations" might be needed outside of "checkout".
        OperationEnvironmentImpl env(desc);
        env.setCheckout( this );
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
        std::stringstream filenam;
        filenam << prefix << UPNS_INSTALL_OPERATORS << desc.operatorname() << debug << postfix;
        if(desc.operatorversion())
        {
            filenam << "." << desc.operatorversion();
        }
        std::stringstream fixpathfilenam;
        fixpathfilenam << "./libs/operator_modules_collection/" << desc.operatorname() << "/" << filenam.str();
        std::string filenamestr = fixpathfilenam.str();
        log_info("loading operator module \"" + filenamestr + "\"");
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
                return OperationResult(UPNS_STATUS_ERR_MODULE_OPERATOR_NOT_FOUND, OperationDescription());
            }
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
    else
    {
        std::unique_ptr<upns::RequestOperatorExecution> req(new upns::RequestOperatorExecution);
        req->set_checkout(m_checkoutName);
        *req->mutable_param() = desc;
        m_node->send(std::move(req));
        std::shared_ptr<upns::ReplyOperatorExecution> rep(m_node->receive<upns::ReplyOperatorExecution>());
        upns::OperationResult res;
        res.first = rep->status_code();
        res.second = rep->result();
        if(!rep->error_msg().empty())
        {
            log_error("Remote Operator Executions returned an error: " + rep->error_msg());
        }
        return res;
    }
}

upns::OperationResult upns::ZmqRequesterCheckout::doUntraceableOperation(const upns::OperationDescription &desc, std::function<upns::StatusCode (upns::OperationEnvironment *)> operate)
{
    upns::OperationEnvironmentImpl env( desc );
    env.setCheckout( this );
    // TODO: this breaks req/resp pattern!
    upns::StatusCode status = operate( &env );
    OperationResult res(status, env.outputDescription());
    return res;
}

//void upns::ZmqRequesterCheckout::syncHierarchy()
//{
//    std::unique_ptr<upns::RequestHierarchy> req(new upns::RequestHierarchy);
//    req->set_checkout(m_checkoutName);
//    m_node->send(std::move(req));
//    std::unique_ptr<upns::ReplyHierarchy> hierarchy(m_node->receive<upns::ReplyHierarchy>());
//    assert(m_cache);
//    //m_cache->
//}

upns::StatusCode upns::ZmqRequesterCheckout::storeTree(const upns::Path &path, std::shared_ptr<upns::Tree> tree)
{
    return UPNS_STATUS_ERR_NOT_YET_IMPLEMENTED;
}

upns::StatusCode upns::ZmqRequesterCheckout::storeEntity(const upns::Path &path, std::shared_ptr<upns::Entity> entity)
{
    std::unique_ptr<upns::RequestStoreEntity> req(new upns::RequestStoreEntity);
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    req->set_type( entity->type() );
    req->set_offset(0ul);
    req->set_sendlength(0ul);
    req->set_entitylength(0ul);
    m_node->send(std::move(req));
    std::shared_ptr<upns::ReplyStoreEntity> rep(m_node->receive<upns::ReplyStoreEntity>());
    if(rep->status() == upns::ReplyStoreEntity::SUCCESS)
    {
        return UPNS_STATUS_OK;
    }
    else
    {
        log_error("Could not store entity \"" + path + "\"");
        return UPNS_STATUS_ERROR;
    }
}

std::shared_ptr<upns::AbstractEntitydata> upns::ZmqRequesterCheckout::getEntitydataForReadWrite(const upns::Path &entity)
{
    std::shared_ptr<Entity> e = getEntity(entity);
    if(!e)
    {
        log_error("Entity could not be queried for entitydata: " + entity);
        return std::shared_ptr<upns::AbstractEntitydata>(nullptr);
    }
    std::shared_ptr<AbstractEntitydataProvider> streamProvider(new ZmqEntitydataStreamProvider(m_checkoutName, entity, m_node));
    return EntityDataLibraryManager::getEntitydataFromProvider(e->type(), streamProvider);
}
