#include "upnszmqrequestercheckout.h"
#include "services_internal.pb.h"
#include "serialization/entitystreammanager.h" //TODO: put in yet another independent project/cmake target. No dependecy to mapmanager required.
#include "serialization/zmqentitydatastreamprovider.h"
#include "operationenvironmentimpl.h"
#include "upns_errorcodes.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#include "module.h"
typedef ModuleInfo* (*GetModuleInfo)();

upns::ZmqRequesterCheckout::ZmqRequesterCheckout(upnsString name, ZmqProtobufNode *node, Checkout *cache, bool operationsLocal)
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

upns::upnsVec<upns::upnsSharedPointer<upns::Conflict> > upns::ZmqRequesterCheckout::getPendingConflicts()
{
    //TODO: nyi
    assert(false);
    return upns::upnsVec<upns::upnsSharedPointer<upns::Conflict> >();
}

void upns::ZmqRequesterCheckout::setConflictSolved(const upns::Path &path, const upns::ObjectId &oid)
{
    //TODO: nyi
    assert(false);
}

upns::upnsSharedPointer<upns::Tree> upns::ZmqRequesterCheckout::getRoot()
{
    std::unique_ptr<upns::RequestHierarchy> req(new upns::RequestHierarchy);
    req->set_checkout(m_checkoutName);
    m_node->send(std::move(req));
    std::unique_ptr<upns::ReplyHierarchy> rep(m_node->receive<upns::ReplyHierarchy>());
    upns::upnsSharedPointer<upns::Tree> ret(new upns::Tree);
    for(google::protobuf::Map< ::std::string, ::upns::ReplyHierarchyMap >::const_iterator ch( rep->maps().cbegin() );
        ch != rep->maps().cend();
        ++ch)
    {
        ret->mutable_refs()->insert(::google::protobuf::MapPair< ::std::string, ::upns::ObjectReference>(ch->first, upns::ObjectReference()));
    }
    return ret;
}

upns::upnsSharedPointer<upns::Tree> upns::ZmqRequesterCheckout::getTreeConflict(const upns::ObjectId &objectId)
{
    //TODO: nyi
    assert(false);
    return upns::upnsSharedPointer<upns::Tree>();
}

upns::upnsSharedPointer<upns::Entity> upns::ZmqRequesterCheckout::getEntityConflict(const upns::ObjectId &objectId)
{
    //TODO: nyi
    assert(false);
    return upns::upnsSharedPointer<upns::Entity>();
}

upns::upnsSharedPointer<upns::Tree> upns::ZmqRequesterCheckout::getTree(const upns::Path &path)
{
    std::unique_ptr<upns::RequestTree> req(new upns::RequestTree);
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    m_node->send(std::move(req));
    upns::upnsSharedPointer<upns::ReplyTree> rep(m_node->receive<upns::ReplyTree>());
    upns::upnsSharedPointer<upns::Tree> ret(rep->release_tree());
    return ret;
}

upns::upnsSharedPointer<upns::Entity> upns::ZmqRequesterCheckout::getEntity(const upns::Path &path)
{
    std::unique_ptr<upns::RequestEntity> req(new upns::RequestEntity);
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    m_node->send(std::move(req));
    upns::upnsSharedPointer<upns::ReplyEntity> rep(m_node->receive<upns::ReplyEntity>());
    upns::upnsSharedPointer<upns::Entity> ret(rep->release_entity());
    return ret;
}

upns::upnsSharedPointer<upns::Branch> upns::ZmqRequesterCheckout::getParentBranch()
{
    //TODO: nyi
    assert(false);
    return upns::upnsSharedPointer<upns::Branch>();
}

upns::upnsVec<upns::CommitId> upns::ZmqRequesterCheckout::getParentCommitIds()
{
    //TODO: nyi
    assert(false);
    return upns::upnsVec<upns::CommitId>();
}

upns::upnsSharedPointer<upns::AbstractEntitydata> upns::ZmqRequesterCheckout::getEntitydataReadOnly(const upns::Path &entityId)
{
    upnsSharedPointer<Entity> e = getEntity(entityId);
    if(!e)
    {
        log_error("Entity could not be queried for entitydata: " + entityId);
        return upns::upnsSharedPointer<upns::AbstractEntitydata>(nullptr);
    }
    upns::upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider(new ZmqEntitydataStreamProvider(m_checkoutName, entityId, m_node));
    return EntityStreamManager::getEntitydataFromStreamImpl(e->type(), streamProvider);
}

upns::upnsSharedPointer<upns::AbstractEntitydata> upns::ZmqRequesterCheckout::getEntitydataReadOnlyConflict(const upns::ObjectId &entityId)
{
    //TODO: nyi
    assert(false);
    return upns::upnsSharedPointer<upns::AbstractEntitydata>(nullptr);
}

upns::StatusCode upns::ZmqRequesterCheckout::depthFirstSearch(std::function<bool (upns::upnsSharedPointer<upns::Commit>, const upns::ObjectId &, const upns::Path &)> beforeCommit, std::function<bool (upns::upnsSharedPointer<upns::Commit>, const upns::ObjectId &, const upns::Path &)> afterCommit, std::function<bool (upns::upnsSharedPointer<upns::Tree>, const upns::ObjectId &, const upns::Path &)> beforeTree, std::function<bool (upns::upnsSharedPointer<upns::Tree>, const upns::ObjectId &, const upns::Path &)> afterTree, std::function<bool (upns::upnsSharedPointer<upns::Entity>, const upns::ObjectId &, const upns::Path &)> beforeEntity, std::function<bool (upns::upnsSharedPointer<upns::Entity>, const upns::ObjectId &, const upns::Path &)> afterEntity)
{
    //TODO: nyi
    assert(false);
    return UPNS_STATUS_ERR_NOT_YET_IMPLEMENTED;
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
        upnsString debug(DEBUG_POSTFIX);
    #else
        upnsString debug("");
    #endif

    #ifdef _WIN32
        upnsString prefix("");
        upnsString postfix(".dll");
    #else
        upnsString prefix("lib");
        upnsString postfix(".so");
    #endif
        std::stringstream filename;
        filename << "./libs/operator_modules_collection/" << desc.operatorname() << "/" << prefix << desc.operatorname() << debug << postfix;
        if(desc.operatorversion())
        {
            filename << "." << desc.operatorversion();
        }
        std::string filenamestr = filename.str();
        log_info("loading operator module \"" + filenamestr + "\"");
    #ifdef _WIN32
        HMODULE handle = LoadLibrary(filename.str().c_str());
    #else
        void* handle = dlopen(filenamestr.c_str(), RTLD_NOW);
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
    else
    {
        std::unique_ptr<upns::RequestOperatorExecution> req(new upns::RequestOperatorExecution);
        req->set_checkout(m_checkoutName);
        *req->mutable_param() = desc;
        m_node->send(std::move(req));
        upns::upnsSharedPointer<upns::ReplyOperatorExecution> rep(m_node->receive<upns::ReplyOperatorExecution>());
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

void upns::ZmqRequesterCheckout::syncHierarchy()
{
    std::unique_ptr<upns::RequestHierarchy> req(new upns::RequestHierarchy);
    req->set_checkout(m_checkoutName);
    m_node->send(std::move(req));
    std::unique_ptr<upns::ReplyHierarchy> hierarchy(m_node->receive<upns::ReplyHierarchy>());
    assert(m_cache);
    //m_cache->

}

upns::StatusCode upns::ZmqRequesterCheckout::storeTree(const upns::Path &path, upnsSharedPointer<upns::Tree> tree)
{
    return UPNS_STATUS_ERR_NOT_YET_IMPLEMENTED;
}

upns::StatusCode upns::ZmqRequesterCheckout::storeEntity(const upns::Path &path, upnsSharedPointer<upns::Entity> entity)
{
    std::unique_ptr<upns::RequestStoreEntity> req(new upns::RequestStoreEntity);
    req->set_checkout(m_checkoutName);
    req->set_path(path);
    req->set_type( entity->type() );
    req->set_offset(0ul);
    req->set_sendlength(0ul);
    req->set_entitylength(0ul);
    m_node->send(std::move(req));
    upns::upnsSharedPointer<upns::ReplyStoreEntity> rep(m_node->receive<upns::ReplyStoreEntity>());
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

upns::upnsSharedPointer<upns::AbstractEntitydata> upns::ZmqRequesterCheckout::getEntitydataForReadWrite(const upns::Path &entity)
{
    upnsSharedPointer<Entity> e = getEntity(entity);
    if(!e)
    {
        log_error("Entity could not be queried for entitydata: " + entity);
        return upns::upnsSharedPointer<upns::AbstractEntitydata>(nullptr);
    }
    upns::upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider(new ZmqEntitydataStreamProvider(m_checkoutName, entity, m_node));
    return EntityStreamManager::getEntitydataFromStreamImpl(e->type(), streamProvider);
}
