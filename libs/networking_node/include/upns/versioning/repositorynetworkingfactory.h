#ifndef REPOSITORYNETWOKINGFACTORY_H
#define REPOSITORYNETWOKINGFACTORY_H

#include <upns/typedefs.h>
#include <upns/versioning/repository.h>
#include "upns/repositoryserver.h"

namespace upns
{

/**
 * @brief The RepositoryNetworkingFactory class
 * Returned Repositories must be deleted manually. Caller has ownership.
 * Deleting the object does NOT delete the repositories contents from disk.
 */

class RepositoryNetworkingFactory
{
public:

    /**
     * @brief openRepositoryAsServer. Creates a network connected to a given repository.
     * This creates a shim network-repository, which conntects to a second local repository.
     * Requests over the network will be redirected to the local repository.
     * Blocking call. Can only be stopped by sigint/sigterm at the moment
     * @return
     */
    static RepositoryServer* openRepositoryAsServer(const int port, upns::Repository* repo, upns::upnsString urlNext = upns::upnsString() );

    /**
     * @brief connectToRemoteRepository. Creates a shim repository, which delegates all requests to a repository in the network. It also handles replies.
     * This can be used just like a normal repository, but method calls may take longer until completion.
     * Note: Calls should not be called in a main or gui thread, as this would block messages from OS and freeze UIs.
     * //TODO: Rename to clone/...
     * @return
     */
    static upns::Repository* connectToRemoteRepository(upns::upnsString url, Repository* cache, bool operationsLocal = false);
};

}
#endif
