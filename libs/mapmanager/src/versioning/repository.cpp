#include "versioning/repository.h"
#include "yaml-cpp/yaml.h"
#include "serialization/abstractmapserializer.h"
#include "serialization/leveldb/leveldbserializer.h"
#include "versioning/checkoutimpl.h"
#include "serialization/entitystreammanager.h"

namespace upns
{
class RepositoryPrivate
{
    RepositoryPrivate():m_serializer(NULL){}
    AbstractMapSerializer* m_serializer;

    void initialize()
    {
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

Repository::Repository(const YAML::Node &config)
    :m_p(new RepositoryPrivate)
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
                m_p->m_serializer = new LevelDBSerializer(mapsource);
            } else {
                log_error("mapsource '" + mapsrcnam + "' was not found.");
            }
        } else {
            log_error("'mapsource' has no 'name' in config");
        }
    } else {
        log_error("Key 'mapsource' not given in config");
    }
    assert(m_p->m_serializer);
    m_p->initialize();
}

Repository::~Repository()
{
    delete m_p;
}

upnsSharedPointer<Checkout> Repository::checkout(const CommitId &commitIdOrBranchname, const upnsString &name)
{
    upnsSharedPointer<CheckoutObj> co(m_p->m_serializer->getCheckoutCommit(name));
    if(co != NULL)
    {
        log_info("Checkout with this name already exist: " + name);
        return NULL;
    }
    upnsSharedPointer<Branch> branch(m_p->m_serializer->getBranch(commitIdOrBranchname));
    CommitId commitId;
    upnsString branchName;
    if(branch != NULL)
    {
        // assert: empty, if this is the inial commit and "master"
        assert( branch->commitid().empty() || m_p->m_serializer->getCommit(branch->commitid()) != NULL );
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
            log_info("given commitIdOrBranchname was not a commitId or branchname.");
            return NULL;
        }
    }
    co = upnsSharedPointer<CheckoutObj>(new CheckoutObj());
    co->mutable_commit()->add_parentcommitids(commitId);
    m_p->m_serializer->createCheckoutCommit( co, name );
    return upnsSharedPointer<Checkout>(new CheckoutImpl(m_p->m_serializer, co, branchName));
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

upnsSharedPointer<CheckoutObj> Repository::getCheckout(const upnsString &name)
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

upnsSharedPointer<Checkout> Repository::checkout(const upnsString &checkoutName)
{
    upnsSharedPointer<CheckoutObj> co(m_p->m_serializer->getCheckoutCommit(checkoutName));
    if(co == NULL)
    {
        log_info("Checkout does not exist: " + checkoutName);
        return NULL;
    }
    return upnsSharedPointer<Checkout>(new CheckoutImpl(m_p->m_serializer, co));
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

CommitId Repository::commit(const upnsSharedPointer<Checkout> checkout, const upnsString msg)
{
    return "";
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
