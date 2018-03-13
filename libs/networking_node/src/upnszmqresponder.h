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

#ifndef UPNSZMQRESPONDER_H
#define UPNSZMQRESPONDER_H

#include <string>
#include "upns/repositoryserver.h"

namespace upns {

class ZmqResponderPrivate;
class Repository;

///
/// \brief The upns::ZmqResponder class
/// Acts as a server and tries to answer requests to a map-repository.
/// If a request can not be answered by this repository, the request may be forwarded to another node.
///

class ZmqResponder : public RepositoryServer
{
public:
    ZmqResponder( int portIncomingRequests, Repository* repo, std::string urlOutgoingRequests = std::string() );
    virtual ~ZmqResponder();

    // RepositoryServer interface
public:
    void handleRequest(int milliseconds);

private:
    ZmqResponderPrivate *m_d;
};

}

#endif // UPNSZMQRESPONDER_H
