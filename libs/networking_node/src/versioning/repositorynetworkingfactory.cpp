#include "versioning/repositorynetworkingfactory.h"
#include "upnszmqrequester.h"
#include "upnszmqresponder.h"
#include "repositoryserver.h"

upns::RepositoryServer* upns::RepositoryNetworkingFactory::openRepositoryAsServer(const int port, upns::Repository *repo, upns::upnsString urlNext)
{
    ZmqResponder* resp = new ZmqResponder(port, repo, urlNext);
    return resp;
}

upns::Repository *upns::RepositoryNetworkingFactory::connectToRemoteRepository(upns::upnsString url, Repository *cache, bool operationsLocal)
{
    //requester
    ZmqRequester* req = new ZmqRequester( cache, url, operationsLocal );
    return req;
}
