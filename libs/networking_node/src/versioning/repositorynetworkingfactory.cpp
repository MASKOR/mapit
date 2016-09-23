#include "versioning/repositorynetworkingfactory.h"
#include "upnszmqrequester.h"
#include "upnszmqresponder.h"
#include "repositoryserver.h"

upns::Repository *upns::RepositoryNetworkingFactory::openLocalRepositoryAsServer(const int port, const upns::upnsString &filename)
{
    // TODO: other library which connects networking and local repo
    assert(false);
    return NULL;
}

upns::Repository *upns::RepositoryNetworkingFactory::openLocalRepositoryAsServer(const int port, const YAML::Node &config)
{
    // TODO: other library which connects networking and local repo
    assert(false);
    return NULL;
}

upns::RepositoryServer* upns::RepositoryNetworkingFactory::openRepositoryAsServer(const int port, upns::Repository *repo, upns::upnsString urlNext)
{
    ZmqResponder* resp = new ZmqResponder(port, repo, urlNext);
    return resp;
}

upns::Repository *upns::RepositoryNetworkingFactory::connectToRemoteRepository(upns::upnsString url)
{
    //requester
    ZmqRequester* req = new ZmqRequester( url );
    return req;
}

upns::Repository *upns::RepositoryNetworkingFactory::connectToRemoteRepositoryCached(upns::upnsString url, const upnsString filename)
{
    // TODO: other library which connects networking and local repo
    assert(false);
    return NULL;
}

upns::Repository *upns::RepositoryNetworkingFactory::connectToRemoteRepositoryCached(upns::upnsString url, const YAML::Node &config)
{
    // TODO: other library which connects networking and local repo
    assert(false);
    return NULL;
}
