#include "repositoryimpl.h"
#include "yaml-cpp/yaml.h"
#include "serialization/abstractmapserializer.h"
#include "versioning/checkoutimpl.h"
#include "serialization/entitystreammanager.h"
#include <chrono>

namespace upns
{
class RepositoryPrivate
{
    RepositoryPrivate(upns::upnsSharedPointer<AbstractMapSerializer> ser):m_serializer(ser){}

    upns::upnsSharedPointer<AbstractMapSerializer> m_serializer;
    friend class RepositoryImpl;
};

RepositoryImpl::RepositoryImpl(upns::upnsSharedPointer<AbstractMapSerializer> serializer)
    :m_p(new RepositoryPrivate(serializer))
{}

RepositoryImpl::~RepositoryImpl()
{
    delete m_p;
}

upnsSharedPointer<Checkout> RepositoryImpl::createCheckout(const CommitId &commitIdOrBranchname, const upnsString &name)
{
    upnsSharedPointer<CheckoutObj> co(m_p->m_serializer->getCheckoutCommit(name));
    if(co != NULL)
    {
        log_error("Checkout with this name already exist: " + name);
        return NULL;
    }
    upnsSharedPointer<Branch> branch(m_p->m_serializer->getBranch(commitIdOrBranchname));
    CommitId commitId;
    upnsString branchName;
    if(branch != NULL)
    {
        // assert: empty, if this is the inial commit and "master"
        assert( branch->commitid().empty() || m_p->m_serializer->getCommit(branch->commitid()) != NULL );
        if(branch->commitid().empty())
        {
            log_info("empty repository. checking out initial master");
        }
        commitId = branch->commitid();
        branchName = commitIdOrBranchname;
    }
    else
    {
        upnsSharedPointer<Commit> commit(m_p->m_serializer->getCommit(commitIdOrBranchname));
        if(commit != NULL)
        {
            commitId = commitIdOrBranchname;
            branchName = "";
        }
        else
        {
            log_error("given commitIdOrBranchname was not a commitId or branchname.");
            return NULL;
        }
    }
    co = upnsSharedPointer<CheckoutObj>(new CheckoutObj());
    co->mutable_rollingcommit()->add_parentcommitids(commitId);
    StatusCode s = m_p->m_serializer->createCheckoutCommit( co, name );
    if(!upnsIsOk(s))
    {
        log_error("Could not create checkout.");
    }
    return upnsSharedPointer<Checkout>(new CheckoutImpl(m_p->m_serializer, co, name, branchName));
}

upnsVec<upnsString> RepositoryImpl::listCheckoutNames()
{
    return m_p->m_serializer->listCheckoutNames();
}

upnsSharedPointer<Tree> RepositoryImpl::getTree(const ObjectId &oid)
{
    return m_p->m_serializer->getTree(oid);
}

upnsSharedPointer<Entity> RepositoryImpl::getEntity(const ObjectId &oid)
{
    return m_p->m_serializer->getEntity(oid);
}

upnsSharedPointer<Commit> RepositoryImpl::getCommit(const ObjectId &oid)
{
    return m_p->m_serializer->getCommit(oid);
}

upnsSharedPointer<CheckoutObj> RepositoryImpl::getCheckoutObj(const upnsString &name)
{
    return m_p->m_serializer->getCheckoutCommit(name);
}

upnsSharedPointer<Branch> RepositoryImpl::getBranch(const upnsString &name)
{
    return m_p->m_serializer->getBranch(name);
}

MessageType RepositoryImpl::typeOfObject(const ObjectId &oid)
{
    // TODO fix this for transient and then not transient
    return m_p->m_serializer->typeOfObject(oid);
}

upnsSharedPointer<AbstractEntitydata> RepositoryImpl::getEntitydataReadOnly(const ObjectId &oid)
{
    // For entitydata it is not enough to call serializer directly.
    // Moreover special classes need to be created by layertype plugins.
    upnsSharedPointer<Entity> ent = m_p->m_serializer->getEntity( oid );
    if( ent == NULL )
    {
        log_error("Entity not found." + oid);
        return NULL;
    }
    assert( ent );
    return EntityStreamManager::getEntitydataFromStreamImpl(ent->type(), m_p->m_serializer->getStreamProvider(oid, true), true);
}

upnsSharedPointer<Checkout> RepositoryImpl::getCheckout(const upnsString &checkoutName)
{
    upnsSharedPointer<CheckoutObj> co(m_p->m_serializer->getCheckoutCommit(checkoutName));
    if(co == NULL)
    {
        log_info("Checkout does not exist: " + checkoutName);
        return NULL;
    }
    return upnsSharedPointer<Checkout>(new CheckoutImpl(m_p->m_serializer, co, checkoutName));
}

StatusCode RepositoryImpl::deleteCheckoutForced(const upnsString &checkoutName)
{
    upnsSharedPointer<CheckoutObj> co(m_p->m_serializer->getCheckoutCommit(checkoutName));
    if(co == NULL)
    {
        log_info("Checkout with this name does not exist: " + checkoutName);
        return UPNS_STATUS_ENTITY_NOT_FOUND;
    }
    //TODO: Get Checkout, remove its inner Commit and objects (objects only if not referenced)!
    return m_p->m_serializer->removeCheckoutCommit(checkoutName);
}

CommitId RepositoryImpl::commit(const upnsSharedPointer<Checkout> checkout, upnsString msg)
{
    CheckoutImpl *co = static_cast<CheckoutImpl*>(checkout.get());
    std::map< std::string, std::string > oldToNewIds;
    CommitId ret;
    StatusCode s = co->depthFirstSearch(
        [&](upnsSharedPointer<Commit> obj, const ObjectId& oid, const Path& p){return true;}, [&](upnsSharedPointer<Commit> obj, const ObjectId& oid, const Path& p)
        {
            std::string rootId(obj->root());
            assert(rootId.empty() || oldToNewIds.find(rootId) != oldToNewIds.end());
            if(rootId.empty() && oldToNewIds.find(rootId) != oldToNewIds.end())
            {
                log_warn("commit empty checkout on empty parent commit (no root)");
            }
            else
            {
                obj->set_root(oldToNewIds[rootId]);
            }
            //TODO: Lots of todos here (Metadata)
            if(msg.find_last_of('\n') != msg.length()-1)
            {
                msg += "\n";
            }
            obj->set_commitmessage(msg);
            int64_t millisecs = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
            obj->set_datetime(millisecs);
            obj->set_author("tester <test@maskor.fh-aachen.de>");

            upnsPair<StatusCode, ObjectId> soid = m_p->m_serializer->createCommit(obj);
            if(upnsIsOk(!soid.first)) return false;
            ret = soid.second;
            return true;
        },
        [&](upnsSharedPointer<Tree> obj, const ObjectId& oid, const Path& p){return true;}, [&](upnsSharedPointer<Tree> obj, const ObjectId& oid, const Path& p)
        {
            assert(obj != NULL);
            ::google::protobuf::Map< ::std::string, ::upns::ObjectReference > &refs = *obj->mutable_refs();
            ::google::protobuf::Map< ::std::string, ::upns::ObjectReference >::iterator iter(refs.begin());
            while(iter != refs.end())
            {
                ::std::string id(iter->second.id());
                assert(oldToNewIds.find(id) != oldToNewIds.end());
                iter->second.set_id(oldToNewIds[id]);
                iter++;
            }
            upnsPair<StatusCode, ObjectId> soid = m_p->m_serializer->storeTree(obj);
            if(upnsIsOk(!soid.first)) return false;
            oldToNewIds.insert(std::pair<std::string, std::string>(oid, soid.second));
            return true;
        },
        [&](upnsSharedPointer<Entity> obj, const ObjectId& oid, const Path& p){return true;}, [&](upnsSharedPointer<Entity> obj, const ObjectId& oid, const Path& p)
        {
            upnsPair<StatusCode, ObjectId> soid = m_p->m_serializer->persistTransientEntitydata(oid);
            if(upnsIsOk(!soid.first)) return false;
            //TODO: Put old->New for entitydata (How?!?)
            //oldToNewIds.insert(oid, soid.second);
            obj->set_dataid(soid.second);
            soid = m_p->m_serializer->storeEntity(obj);
            if(upnsIsOk(!soid.first)) return false;
            oldToNewIds.insert(std::pair<std::string, std::string>(oid, soid.second));
            return true;
        });
    if(!upnsIsOk(s))
    {
        log_error("error while commiting");
    }
    //TODO: What to do with old ids?
    ::upns::Commit *ci = co->getCheckoutObj()->mutable_rollingcommit();
    ci->clear_author();
    ci->clear_commitmessage();
    ci->clear_datetime();
    ci->clear_detailedtransitions();
    ci->clear_ops();
    ci->clear_parentcommitids();
    //ci->clear_root();
    ci->clear_transitions();
    ci->add_parentcommitids(ret);
    upnsSharedPointer<CheckoutObj> obj(co->getCheckoutObj());
    s = m_p->m_serializer->storeCheckoutCommit(obj, co->getName());
    if(!upnsIsOk(s))
    {
        log_error("Error during commit. Could not update current checkout.");
    }
    //m_p->m_serializer->debugDump();
    return ret;
}

upnsVec<upnsSharedPointer<Branch> > RepositoryImpl::getBranches()
{
    return upnsVec<upnsSharedPointer<Branch> >();
}

StatusCode RepositoryImpl::push(Repository &repo)
{
    return UPNS_STATUS_OK;
}

StatusCode RepositoryImpl::pull(Repository &repo)
{
    return UPNS_STATUS_OK;
}

CommitId RepositoryImpl::parseCommitRef(const upnsString &commitRef)
{
    return "";
}

upnsSharedPointer<Checkout> RepositoryImpl::merge(const CommitId mine, const CommitId theirs, const CommitId base)
{
    return NULL;
}

upnsVec<upnsPair<CommitId, ObjectId> > RepositoryImpl::ancestors(const CommitId &commitId, const ObjectId &objectId, const int level)
{
    return upnsVec<upnsPair<CommitId, ObjectId> >();
}

bool RepositoryImpl::canRead()
{
    return true;
}

bool RepositoryImpl::canWrite()
{
    return true;
}

}
