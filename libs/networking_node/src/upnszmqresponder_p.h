/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef UPNSZMQRESPONDER_P_H
#define UPNSZMQRESPONDER_P_H

#include <string>
#include "zmqprotobufnode.h"
#include <mapit/msgs/services.pb.h>
#include <mapit/msgs/services_internal.pb.h>

namespace mapit {

class Repository;

/**
 * @brief The ZmqResponderPrivate class Pimpl-Pattern. Hides details from include-header and guarantees compatibility across minor versions.
 */

class ZmqResponderPrivate : public ZmqProtobufNode
{

public:
    ZmqResponderPrivate( int portIncomingRequests, Repository* repo, std::string urlOutgoingRequests = std::string() );
    void handleRequestCheckout(RequestCheckout* msg);
    void handleRequestEntitydata(RequestEntitydata* msg);
    void handleRequestHierarchy(RequestHierarchy* msg);
    void handleRequestHierarchyPlain(RequestHierarchyPlain* msg);
    void handleRequestListCheckouts(RequestListCheckouts* msg);
    void handleRequestOperatorExecution(RequestOperatorExecution* msg);
    void handleRequestGenericEntry(RequestGenericEntry *msg);
    void handleRequestStoreEntity(RequestStoreEntity* msg);
//    void handleRequestStoreTree(RequestStoreTree *msg);
    void handleRequestDeleteEntity(RequestDeleteEntity* msg);
    void handleRequestDeleteTree(RequestDeleteTree* msg);
//    void handleRequestStoreGenericEntry(mapit::RequestStoreGenericEntry *msg);
//    void handleRequestEntity(mapit::RequestEntity *msg);
//    void handleRequestTree(mapit::RequestTree *msg);

    Repository *m_repo;
    std::string m_urlOutgoing;
    int m_portIncoming;

    // Implementation can be in cpp, because it's only used there. Must be public for function pointers
    template < typename T, void (mapit::ZmqResponderPrivate::*func)(T*) >
    void toDelegate(google::protobuf::Message* msg);

private:
    friend class ZmqResponder;
};

}

#endif // UPNSZMQRESPONDER_P_H
