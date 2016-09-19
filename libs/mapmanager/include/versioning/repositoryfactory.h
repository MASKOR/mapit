#ifndef REPOSITORYFACTORY_H
#define REPOSITORYFACTORY_H

#include "upns_globals.h"
#include "services.pb.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include "entitydata.h"
#include "checkout.h"
#include "yaml-cpp/yaml.h"
#include "repository.h"

namespace upns
{
class RepositoryFactory
{
public:
    /**
     * @brief openLocalRepository. Opens a Repository on disc or creates an empty repository.
     * @param filename of yaml config
     * It communicates directly to file serializer.
     * @return
     */
    static upns::Repository* openLocalRepository(const upnsString &filename);

    /**
     * @brief openLocalRepository. Opens a Repository on disc or creates an empty repository.
     * @param config Yaml configuration of the repository
     * It communicates directly to file serializer.
     * @return
     */
    static upns::Repository* openLocalRepository(const YAML::Node &config);

    /**
     * @brief openLocalRepositoryAsServer. Creates a network-repository connected to a local repository.
     * This creates two instances of the repository class. A shim network-repository, which conntects to a second local repository.
     * Requests over the network will be redirected to the local repository.
     * @return
     */
    static upns::Repository* openLocalRepositoryAsServer();

    /**
     * @brief openRemoteRepository. Creates a shim repository, which delegates all requests to a repository in the network. It also handles replies.
     * This can be used just like a normal repository, but method calls may take longer until completion.
     * Note: Calls should not be called in a main or gui thread, as this would block messages from OS and freeze UIs.
     * @return
     */
    static upns::Repository* openRemoteRepository();
};

}
#endif
