#include "versioning/repository.h"
#include "yaml-cpp/yaml.h"
#include "serialization/abstractmapserializer.h"
#include "serialization/leveldb/leveldbserializer.h"
#include "serialization/file_system/fs_serializer.h"
#include "versioning/checkoutimpl.h"
#include "serialization/entitystreammanager.h"
#include <QHash>
#include <QMap>

namespace upns
{
class RepositoryPrivate
{
    RepositoryPrivate():m_serializer(NULL){}
    ~RepositoryPrivate()
    {
        delete m_serializer;
    }

    AbstractMapSerializer* m_serializer;

    void initialize(const YAML::Node &config)
    {
        if(const YAML::Node mapsource = config["mapsource"])
        {
            if(const YAML::Node mapsourceName = mapsource["name"])
            {
                std::string mapsrcnam = mapsourceName.as<std::string>();
                std::transform(mapsrcnam.begin(), mapsrcnam.end(), mapsrcnam.begin(), ::tolower);
                AbstractMapSerializer *mser = NULL;
                if(mapsrcnam == "mapfileservice")
                {
                    //m_serializer = new LevelDBSerializer(mapsource);
                    m_p->m_serializer = new FSSerializer(mapsource);
                } else {
                    log_error("mapsource '" + mapsrcnam + "' was not found.");
                }
            } else {
                log_error("'mapsource' has no 'name' in config");
            }
        } else {
            log_error("Key 'mapsource' not given in config");
        }
        assert(m_serializer);
        // Check if anything exists in the database
        // Note: There might be commits or objects which are not recognized here.
        // TODO: forbid to delete last branch for this to work. Checkouts might all be deleted.
        size_t numElems = m_serializer->listBranches().size();
        if(numElems) return;
//        numElems = m_serializer->listCheckoutNames().size();
//        if(numElems) return;
        upnsSharedPointer<Branch> master(new Branch());
        master->set_commitid(""); //< InitialCommit
        m_serializer->createBranch(master, "master");
    }
    friend class Repository;
};

Repository::Repository(const upnsString &filename)
    :m_p(new RepositoryPrivate)
{
    YAML::Node config = YAML::LoadFile(filename);
    m_p->initialize(config);
}

Repository::Repository(const YAML::Node &config)
    :m_p(new RepositoryPrivate)
{
    m_p->initialize(config);
}

Repository::~Repository()
{
    delete m_p;
}

upnsSharedPointer<Checkout> Repository::createCheckout(const CommitId &commitIdOrBranchname, const upnsString &name)
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

upnsVec<upnsString> Repository::listCheckoutNames()
{
    return m_p->m_serializer->listCheckoutNames();
}

upnsSharedPointer<Tree> Repository::getTree(const ObjectId &oid)
{
    return m_p->m_serializer->getTree(oid);
}

upnsSharedPointer<Entity> Repository::getEntity(const ObjectId &oid)
{
    return m_p->m_serializer->getEntity(oid);
}

upnsSharedPointer<Commit> Repository::getCommit(const ObjectId &oid)
{
    return m_p->m_serializer->getCommit(oid);
}

upnsSharedPointer<CheckoutObj> Repository::getCheckoutObj(const upnsString &name)
{
    return m_p->m_serializer->getCheckoutCommit(name);
}

upnsSharedPointer<Branch> Repository::getBranch(const upnsString &name)
{
    return m_p->m_serializer->getBranch(name);
}

MessageType Repository::typeOfObject(const ObjectId &oid)
{
    return m_p->m_serializer->typeOfObject(oid);
}

upnsSharedPointer<AbstractEntityData> Repository::getEntityDataReadOnly(const ObjectId &oid)
{
    // For entitydata it is not enough to call serializer directly.
    // Moreover special classes need to be created by layertype plugins.
    return EntityStreamManager::getEntityDataImpl(m_p->m_serializer, oid, true, false);
}

upnsSharedPointer<Checkout> Repository::getCheckout(const upnsString &checkoutName)
{
    upnsSharedPointer<CheckoutObj> co(m_p->m_serializer->getCheckoutCommit(checkoutName));
    if(co == NULL)
    {
        log_info("Checkout does not exist: " + checkoutName);
        return NULL;
    }
    return upnsSharedPointer<Checkout>(new CheckoutImpl(m_p->m_serializer, co, checkoutName));
}

StatusCode Repository::deleteCheckoutForced(const upnsString &checkoutName)
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

CommitId Repository::commit(const upnsSharedPointer<Checkout> checkout, upnsString msg)
{
    CheckoutImpl *co = static_cast<CheckoutImpl*>(checkout.get());
    QMap< ::std::string, ::std::string> oldToNewIds;
    CommitId ret;
    StatusCode s = co->depthFirstSearch(
        [&](upnsSharedPointer<Commit> obj, const ObjectId& oid, const Path& p){return true;}, [&](upnsSharedPointer<Commit> obj, const ObjectId& oid, const Path& p)
        {
            ::std::string rootId(obj->root());
            assert(rootId.empty() || oldToNewIds.contains(rootId));
            if(rootId.empty() && oldToNewIds.contains(rootId))
            {
                log_warn("commit empty checkout on empty parent commit (no root)");
            }
            else
            {
                obj->set_root(oldToNewIds.value(rootId));
            }
            //TODO: Lots of todos here (Metadata)
            if(msg.find_last_of('\n') != msg.length()-1)
            {
                msg += "\n";
            }
            obj->set_commitmessage(msg);
            obj->set_datetime(QDateTime::currentDateTime().toMSecsSinceEpoch());
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
                assert(oldToNewIds.contains(id));
                iter->second.set_id(oldToNewIds.value(id));
                iter++;
            }
            upnsPair<StatusCode, ObjectId> soid = m_p->m_serializer->storeTree(obj);
            if(upnsIsOk(!soid.first)) return false;
            oldToNewIds.insert(oid, soid.second);
            return true;
        },
        [&](upnsSharedPointer<Entity> obj, const ObjectId& oid, const Path& p){return true;}, [&](upnsSharedPointer<Entity> obj, const ObjectId& oid, const Path& p)
        {
            upnsPair<StatusCode, ObjectId> soid = m_p->m_serializer->persistTransientEntityData(oid);
            if(upnsIsOk(!soid.first)) return false;
            //TODO: Put old->New for entitydata (How?!?)
            //oldToNewIds.insert(oid, soid.second);
            obj->set_dataid(soid.second);
            soid = m_p->m_serializer->storeEntity(obj);
            if(upnsIsOk(!soid.first)) return false;
            oldToNewIds.insert(oid, soid.second);
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

upnsVec<upnsSharedPointer<Branch> > Repository::getBranches()
{
    return upnsVec<upnsSharedPointer<Branch> >();
}

StatusCode Repository::push(Repository &repo)
{
    return UPNS_STATUS_OK;
}

StatusCode Repository::pull(Repository &repo)
{
    return UPNS_STATUS_OK;
}

CommitId Repository::parseCommitRef(const upnsString &commitRef)
{
    return "";
}

upnsSharedPointer<Checkout> Repository::merge(const CommitId mine, const CommitId theirs, const CommitId base)
{
    return NULL;
}

upnsVec<upnsPair<CommitId, ObjectId> > Repository::ancestors(const CommitId &commitId, const ObjectId &objectId, const int level)
{
    return upnsVec<upnsPair<CommitId, ObjectId> >();
}

bool Repository::canRead()
{
    return true;
}

bool Repository::canWrite()
{
    return true;
}

}
