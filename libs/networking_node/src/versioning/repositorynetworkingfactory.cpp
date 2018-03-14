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

#include "mapit/versioning/repositorynetworkingfactory.h"
#include "zmqrequester.h"
#include "zmqresponder.h"
#include "mapit/repositoryserver.h"

mapit::RepositoryServer* mapit::RepositoryNetworkingFactory::openRepositoryAsServer(const int port, mapit::Repository *repo, std::string urlNext)
{
    ZmqResponder* resp = new ZmqResponder(port, repo, urlNext);
    return resp;
}

mapit::Repository *mapit::RepositoryNetworkingFactory::connectToRemoteRepository(std::string url, Repository *cache, bool operationsLocal)
{
    //requester
    ZmqRequester* req = new ZmqRequester( cache, url, operationsLocal );
    return req;
}
