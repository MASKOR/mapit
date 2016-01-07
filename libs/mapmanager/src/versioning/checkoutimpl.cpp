#include "checkoutimpl.h"
#include <dlfcn.h>

namespace upns
{


CheckoutImpl::CheckoutImpl(AbstractMapSerializer *serializer, const CommitId commitOrCheckoutId)
    :m_serializer(serializer)
{
    if(m_serializer->isCheckout(commitOrCheckoutId))
    {
        m_checkoutId = commitOrCheckoutId;
    }
    else if(m_serializer->isCommit(commitOrCheckoutId))
    {
        upnsSharedPointer<Commit> co = new Commit();
        co->add_parentcommitids(commitOrCheckoutId);
        m_checkoutId = m_serializer->createCheckoutCommit();
    }
}

CheckoutImpl::~CheckoutImpl()
{
}

bool CheckoutImpl::isInConflictMode()
{

}

upnsVec<upnsSharedPointer<Conflict> > CheckoutImpl::getPendingConflicts()
{

}

upnsSharedPointer<Tree> CheckoutImpl::getRoot()
{

}

upnsSharedPointer<Tree> CheckoutImpl::getChild(ObjectId objectId)
{

}

upnsSharedPointer<AbstractEntityData> CheckoutImpl::getEntityDataImpl(const ObjectId &entityId, bool readOnly)
{
    upnsSharedPointer<Entity> ent = m_serializer->getEntity( entityId );
    if( ent == NULL )
    {
        log_error("Entity not found." + entityId);
        return NULL;
    }
    assert( ent );
    upnsSharedPointer<AbstractEntityData> edata = wrapEntityOfType( ent->type(), m_serializer->getStreamProvider(entityId, readOnly) ) ;
    return edata;
}

upnsSharedPointer<AbstractEntityData> CheckoutImpl::wrapEntityOfType(LayerType type,
                                                                   upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
{
    upnsString layerName;
    switch(type)
    {
    case POINTCLOUD2:
    {
        layerName = "layertype_pointcloud2";
        break;
    }
    default:
        log_error("Unknown layertype: " + std::to_string(type));
        return upnsSharedPointer<AbstractEntityData>(NULL);
    }
    return wrapEntityOfType( layerName, streamProvider );
}

upnsSharedPointer<AbstractEntityData> CheckoutImpl::wrapEntityOfType(upnsString layertypeName,
                                                                  upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider)
{
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
    void* handle = dlopen((upnsString("./layertypes/") + prefix + layertypeName + debug + postfix).c_str(), RTLD_NOW);
    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return upnsSharedPointer<AbstractEntityData>(NULL);
    }
    WrapLayerTypeFunc wrap = (WrapLayerTypeFunc)dlsym(handle, "createEntityData");
    return upnsSharedPointer<AbstractEntityData>( wrap( streamProvider ) );
}

}
