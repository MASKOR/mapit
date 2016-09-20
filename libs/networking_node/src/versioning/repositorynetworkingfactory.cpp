#include "versioning/repositorynetworkingfactory.h"
#include "upnszmqrequester.h"
#include "upnszmqresponder.h"

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

void upns::RepositoryNetworkingFactory::openRepositoryAsServer(const int port, upns::Repository *repo)
{
    //responder
}

upns::Repository *upns::RepositoryNetworkingFactory::connectToRemoteRepository(upns::upnsString &url)
{
    //requester
}

upns::Repository *upns::RepositoryNetworkingFactory::connectToRemoteRepositoryCached(upns::upnsString &url, const upns::upnsString &filename)
{
    // TODO: other library which connects networking and local repo
    assert(false);
    return NULL;
}

upns::Repository *upns::RepositoryNetworkingFactory::connectToRemoteRepositoryCached(upns::upnsString &url, const YAML::Node &config)
{
    // TODO: other library which connects networking and local repo
    assert(false);
    return NULL;
}
