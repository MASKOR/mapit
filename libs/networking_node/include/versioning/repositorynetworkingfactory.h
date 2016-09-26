#ifndef REPOSITORYNETWOKINGFACTORY_H
#define REPOSITORYNETWOKINGFACTORY_H

#include "upns_globals.h"
#include "yaml-cpp/yaml.h"
#include "versioning/repository.h"
#include "repositoryserver.h"

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
     * @brief openLocalRepositoryAsServer. Creates a network-repository connected to a local repository.
     * This creates two instances of the repository class. A shim network-repository, which conntects to a second local repository.
     * Requests over the network will be redirected to the local repository.
     * @return
     */
    static upns::Repository* openLocalRepositoryAsServer(const int port, const upnsString &filename);

    /**
     * @brief openLocalRepositoryAsServer. Creates a network-repository connected to a local repository.
     * This creates two instances of the repository class. A shim network-repository, which conntects to a second local repository.
     * Requests over the network will be redirected to the local repository.
     * @param config Yaml configuration of the repository
     * @return
     */
    static upns::Repository* openLocalRepositoryAsServer(const int port, const YAML::Node &config);

    /**
     * @brief openRepositoryAsServer. Creates a network connected to a given repository.
     * This creates a shim network-repository, which conntects to a second local repository.
     * Requests over the network will be redirected to the local repository.
     * Blocking call. Can only be stopped by sigint/sigterm at the moment
     * @param config Yaml configuration of the repository
     * @return
     */
    static RepositoryServer* openRepositoryAsServer(const int port, upns::Repository* repo, upns::upnsString urlNext = upns::upnsString() );

    /**
     * @brief connectToRemoteRepository. Creates a shim repository, which delegates all requests to a repository in the network. It also handles replies.
     * This can be used just like a normal repository, but method calls may take longer until completion.
     * Note: Calls should not be called in a main or gui thread, as this would block messages from OS and freeze UIs.
     * @return
     */
    static upns::Repository* connectToRemoteRepository(upns::upnsString url, Repository* cache);

    /**
     * @brief connectToRemoteRepositoryCached. Creates a local repository, all requests, that can not be answered locally are requested from remote and are cached locally.
     * @return
     */
    static upns::Repository* connectToRemoteRepositoryCached(upns::upnsString url, const upnsString filename);
    /**
     * @brief connectToRemoteRepositoryCached. Creates a local repository, all requests, that can not be answered locally are requested from remote and are cached locally.
     * @return
     */
    static upns::Repository* connectToRemoteRepositoryCached(upnsString url, const YAML::Node &config);
};

}
#endif
