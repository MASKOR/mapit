#include "repositoryimpl.h"
#include "serialization/abstractserializer.h"
#include "versioning/checkoutimpl.h"
#include <upns/serialization/entitydatalibrarymanager.h>
#include <chrono>

namespace upns
{
class RepositoryPrivate
{
    RepositoryPrivate(std::shared_ptr<AbstractSerializer> ser):m_serializer(ser){}

    std::shared_ptr<AbstractSerializer> m_serializer;
    friend class RepositoryImpl;
};

RepositoryImpl::RepositoryImpl(std::shared_ptr<AbstractSerializer> serializer)
    :m_p(new RepositoryPrivate(serializer))
{}

RepositoryImpl::~RepositoryImpl()
{
    delete m_p;
}

std::shared_ptr<Checkout> RepositoryImpl::createCheckout(const CommitId &commitIdOrBranchname, const std::string &name)
{
    std::shared_ptr<CheckoutObj> co(m_p->m_serializer->getCheckoutCommit(name));
    if(co != NULL)
    {
        log_warn("Checkout with this name already exist: " + name);
        return NULL;
    }
    std::shared_ptr<Branch> branch(m_p->m_serializer->getBranch(commitIdOrBranchname));
    CommitId commitId;
    std::string branchName;
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
        std::shared_ptr<Commit> commit(m_p->m_serializer->getCommit(commitIdOrBranchname));
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
    co = std::shared_ptr<CheckoutObj>(new CheckoutObj());
    co->mutable_rollingcommit()->add_parentcommitids(commitId);
    StatusCode s = m_p->m_serializer->createCheckoutCommit( co, name );
    if(!upnsIsOk(s))
    {
        log_error("Could not create checkout.");
    }
    return std::shared_ptr<Checkout>(new CheckoutImpl(m_p->m_serializer, co, name, branchName));
}

std::vector<std::string> RepositoryImpl::listCheckoutNames()
{
    return m_p->m_serializer->listCheckoutNames();
}

std::shared_ptr<Tree> RepositoryImpl::getTree(const ObjectId &oid)
{
    return m_p->m_serializer->getTree(oid);
}

std::shared_ptr<Entity> RepositoryImpl::getEntity(const ObjectId &oid)
{
    return m_p->m_serializer->getEntity(oid);
}

std::shared_ptr<Commit> RepositoryImpl::getCommit(const ObjectId &oid)
{
    return m_p->m_serializer->getCommit(oid);
}

std::shared_ptr<CheckoutObj> RepositoryImpl::getCheckoutObj(const std::string &name)
{
    return m_p->m_serializer->getCheckoutCommit(name);
}

std::shared_ptr<Branch> RepositoryImpl::getBranch(const std::string &name)
{
    return m_p->m_serializer->getBranch(name);
}

MessageType RepositoryImpl::typeOfObject(const ObjectId &oid)
{
    MessageType type = m_p->m_serializer->typeOfObjectTransient(oid);
    if(type != MessageEmpty) return type;
    return m_p->m_serializer->typeOfObject(oid);
}

std::shared_ptr<AbstractEntitydata> RepositoryImpl::getEntitydataReadOnly(const ObjectId &oid)
{
    // For entitydata it is not enough to call serializer directly.
    // Moreover special classes need to be created by layertype plugins.
    std::shared_ptr<Entity> ent = m_p->m_serializer->getEntity( oid );
    if( ent == NULL )
    {
        log_error("Entity not found." + oid);
        return NULL;
    }
    assert( ent );
    return EntityDataLibraryManager::getEntitydataFromProvider(ent->type(), m_p->m_serializer->getStreamProvider(oid, true), true);
}

std::shared_ptr<Checkout> RepositoryImpl::getCheckout(const std::string &checkoutName)
{
    std::shared_ptr<CheckoutObj> co(m_p->m_serializer->getCheckoutCommit(checkoutName));
    if(co == NULL)
    {
        log_info("Checkout does not exist: " + checkoutName);
        return NULL;
    }
    return std::shared_ptr<Checkout>(new CheckoutImpl(m_p->m_serializer, co, checkoutName));
}

StatusCode RepositoryImpl::deleteCheckoutForced(const std::string &checkoutName)
{
    std::shared_ptr<CheckoutObj> co(m_p->m_serializer->getCheckoutCommit(checkoutName));
    if(co == NULL)
    {
        log_info("Checkout with this name does not exist: " + checkoutName);
        return UPNS_STATUS_ENTITY_NOT_FOUND;
    }
    //TODO: Get Checkout, remove its inner Commit and objects (objects only if not referenced)!
    return m_p->m_serializer->removeCheckoutCommit(checkoutName);
}

CommitId RepositoryImpl::commit(const std::shared_ptr<Checkout> checkout, std::string msg)
{
    CheckoutImpl *co = static_cast<CheckoutImpl*>(checkout.get());
    std::map< Path, ObjectId > oldPathsToNewOids;
    CommitId ret;
    StatusCode s = co->depthFirstSearch(
        [&](std::shared_ptr<Commit> obj, const ObjectReference &ref, const Path &path){return true;},
        [&](std::shared_ptr<Commit> obj, const ObjectReference &ref, const Path &path)
        {
            const Path pathOfRootDir("");
            assert(ref.path().empty() != ref.id().empty()); //XOR
            bool changes = oldPathsToNewOids.find(pathOfRootDir) != oldPathsToNewOids.end();
            // check if there where changes but on a non exclusive object
            assert(!ref.id().empty() && changes);
            if(ref.id().empty() && changes)
            {
                obj->mutable_root()->set_id(oldPathsToNewOids[pathOfRootDir]);
                obj->mutable_root()->clear_path();
            }
            else
            {
                // no changes where found
                log_warn("commit empty checkout on empty parent commit (no root)");
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

            std::pair<StatusCode, ObjectId> statusOid = m_p->m_serializer->createCommit(obj);
            if(upnsIsOk(!statusOid.first)) return false;
            ret = statusOid.second;
            return true;
        },
        [&](std::shared_ptr<Tree> obj, const ObjectReference &ref, const Path &path){return true;},
        [&](std::shared_ptr<Tree> obj, const ObjectReference &ref, const Path &path)
        {
            assert(obj != NULL);
            ::google::protobuf::Map< ::std::string, ::mapit::msgs::ObjectReference > &refs = *obj->mutable_refs();
            ::google::protobuf::Map< ::std::string, ::mapit::msgs::ObjectReference >::iterator iter(refs.begin());
            while(iter != refs.end())
            {
                Path childPath(path + "/" + iter->second.path());
                assert(oldPathsToNewOids.find(childPath) != oldPathsToNewOids.end());
                const ObjectId &newId = oldPathsToNewOids[childPath];
                assert(!newId.empty());
                if(newId != iter->second.id())
                {
                    iter->second.set_id(newId);
                    iter->second.clear_path();
                }
                iter++;
            }
            std::pair<StatusCode, ObjectId> statusOid = m_p->m_serializer->storeTree(obj);
            if(upnsIsOk(!statusOid.first)) return false;
            oldPathsToNewOids.insert(std::pair<std::string, std::string>(path, statusOid.second));
            return true;
        },
        [&](std::shared_ptr<Entity> obj, const ObjectReference &ref, const Path &path){return true;},
        [&](std::shared_ptr<Entity> obj, const ObjectReference &ref, const Path &path)
        {
            std::pair<StatusCode, ObjectId> statusEntitydataOid = m_p->m_serializer->persistTransientEntitydata(ref.path());
            if(upnsIsOk(!statusEntitydataOid.first)) return false;
            bool entityNeedsStore;
            if(statusEntitydataOid.second != obj->dataid())
            {
                obj->set_dataid(statusEntitydataOid.second);
                entityNeedsStore = true;
            }
            else
            {
                assert(ref.path().empty() != ref.id().empty()); //XOR
                entityNeedsStore = !ref.path().empty();
            }

            if(entityNeedsStore)
            {
                std::pair<StatusCode, ObjectId> statusOid = m_p->m_serializer->storeEntity(obj);
                if(upnsIsOk(!statusOid.first)) return false;
                oldPathsToNewOids.insert(std::pair<std::string, std::string>(path, statusOid.second));
            }
            else
            {
                oldPathsToNewOids.insert(std::pair<std::string, std::string>(path, ref.id()));
            }
            //TODO: Put old->New for entitydata (How?!?)
            //oldToNewIds.insert(oid, soid.second);
            return true;
        });
    if(!upnsIsOk(s))
    {
        log_error("error while commiting");
    }
    //TODO: What to do with old ids?
    ::mapit::msgs::Commit *ci = co->getCheckoutObj()->mutable_rollingcommit();
    ci->clear_author();
    ci->clear_commitmessage();
    ci->clear_datetime();
    ci->clear_detailedtransitions();
    ci->clear_ops();
    ci->clear_parentcommitids();
    //ci->clear_root();
    ci->clear_transitions();
    ci->add_parentcommitids(ret);
    std::shared_ptr<CheckoutObj> obj(co->getCheckoutObj());
    s = m_p->m_serializer->storeCheckoutCommit(obj, co->getName());
    if(!upnsIsOk(s))
    {
        log_error("Error during commit. Could not update current checkout.");
    }
    //m_p->m_serializer->debugDump();
    return ret;
}

std::vector<std::shared_ptr<Branch> > RepositoryImpl::getBranches()
{
    return std::vector<std::shared_ptr<Branch> >();
}

StatusCode RepositoryImpl::push(Repository &repo)
{
    return UPNS_STATUS_OK;
}

StatusCode RepositoryImpl::pull(Repository &repo)
{
    return UPNS_STATUS_OK;
}

CommitId RepositoryImpl::parseCommitRef(const std::string &commitRef)
{
    return "";
}

std::shared_ptr<Checkout> RepositoryImpl::merge(const CommitId mine, const CommitId theirs, const CommitId base)
{
    return NULL;
}

std::vector<std::pair<CommitId, ObjectId> > RepositoryImpl::ancestors(const CommitId &commitId, const ObjectId &objectId, const int level)
{
    return std::vector<std::pair<CommitId, ObjectId> >();
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
