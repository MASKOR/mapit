/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef REPOSITORYNETWOKINGFACTORY_H
#define REPOSITORYNETWOKINGFACTORY_H

#include <mapit/typedefs.h>
#include <mapit/versioning/repository.h>
#include "mapit/repositoryserver.h"

namespace mapit
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
    static RepositoryServer* openRepositoryAsServer(const int port, mapit::Repository* repo, std::string urlNext = std::string() );

    /**
     * @brief connectToRemoteRepository. Creates a shim repository, which delegates all requests to a repository in the network. It also handles replies.
     * This can be used just like a normal repository, but method calls may take longer until completion.
     * Note: Calls should not be called in a main or gui thread, as this would block messages from OS and freeze UIs.
     * //TODO: Rename to clone/...
     * @return
     */
    static mapit::Repository* connectToRemoteRepository(std::string url, Repository* cache, bool operationsLocal = false);
};

}
#endif
