#include "versioning/repository.h"
#include "yaml-cpp/yaml.h"
#include "modules/serialization/abstractmapserializerNEW.h"
#include "serialization/leveldb/leveldbserializer.h"
#include "versioning/checkoutimpl.h"
#include "serialization/entitystreammanager.h"

namespace upns
{
class RepositoryPrivate
{
    RepositoryPrivate():m_serializer(NULL){}
    AbstractMapSerializer* m_serializer;
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
}

Repository::~Repository()
{
    delete m_p;
}

upnsSharedPointer<Checkout> Repository::createCheckout(const CommitId &commitId, const upnsString &name)
{
    if(m_p->m_serializer->isCommit(commitId))
    {
        upnsSharedPointer<CheckoutObj> co(upnsSharedPointer<CheckoutObj>(new CheckoutObj()));
        co->mutable_commit()->add_parentcommitids(commitId);
        m_p->m_serializer->createCheckoutCommit( co, name );
        return upnsSharedPointer<Checkout>(new CheckoutImpl(m_p->m_serializer, co));
    }
    return NULL;
}

upnsVec<upnsString> Repository::listCheckoutNames()
{
    return m_p->m_serializer->listCheckoutNames();
}

upnsSharedPointer<Tree> Repository::getTree(const ObjectId &oid)
{
    return m_p->m_serializer->getTree(oid);
}

upnsSharedPointer<Entity> Repository::getEntity(const ObjectId oid)
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
    if(m_p->m_serializer->isCheckout(checkoutName))
    {
        upnsSharedPointer<CheckoutObj> co;
        co = m_p->m_serializer->getCheckoutCommit(checkoutName);
        return upnsSharedPointer<Checkout>(new CheckoutImpl(m_p->m_serializer, co));
    }
    return NULL;
}

StatusCode Repository::deleteCheckoutForced(const upnsString &checkoutName)
{

}

CommitId Repository::commit(const upnsSharedPointer<Checkout> checkout, const upnsString msg)
{

}

upnsVec<upnsSharedPointer<Branch> > Repository::getBranches()
{

}

upnsSharedPointer<Branch> Repository::createBranch(const upnsString &branchname)
{

}

StatusCode Repository::push(Repository &repo)
{

}

StatusCode Repository::pull(Repository &repo)
{

}

CommitId Repository::parseCommitRef(const upnsString &commitRef)
{

}

upnsSharedPointer<Checkout> Repository::merge(const CommitId mine, const CommitId theirs, const CommitId base)
{

}

upnsVec<upnsPair<CommitId, ObjectId> > Repository::ancestors(const CommitId &commitId, const ObjectId &objectId, const int level)
{

}

bool Repository::canRead()
{

}

bool Repository::canWrite()
{

}

}
