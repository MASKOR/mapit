#include "versioning/repository.h"
#include "yaml-cpp/yaml.h"
#include "modules/serialization/abstractmapserializerNEW.h"
#include "serialization/leveldb/leveldbserializer.h"
#include "versioning/checkoutimpl.h"
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

upnsVec<upnsSharedPointer<Checkout> > Repository::getCheckouts()
{

}

upnsSharedPointer<Checkout> Repository::checkout(const upnsSharedPointer<Branch> &commit)
{
    return upnsSharedPointer<Checkout>(new CheckoutImpl(m_p->m_serializer, commit));
}

upnsSharedPointer<Checkout> Repository::checkout(const CommitId &commit)
{
    return upnsSharedPointer<Checkout>(new CheckoutImpl(m_p->m_serializer, commit));
}

StatusCode Repository::deleteCheckoutForced(const upnsSharedPointer<Checkout> checkout)
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
