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

#include "upnszmqresponder.h"
#include "upnszmqresponder_p.h"
#include <upns/versioning/repository.h>
#include "upns/repositoryserver.h"
#include <zmq.hpp>

upns::ZmqResponder::ZmqResponder(int portIncomingRequests, Repository* repo, std::string urlOutgoingRequests)
{
    m_d = new upns::ZmqResponderPrivate(portIncomingRequests, repo, urlOutgoingRequests);
}

upns::ZmqResponder::~ZmqResponder()
{
    delete m_d;
}

void upns::ZmqResponder::handleRequest(int milliseconds)
{
    m_d->receive_and_dispatch(milliseconds);
}

