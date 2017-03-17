#ifndef UPNSZMQRESPONDER_P_H
#define UPNSZMQRESPONDER_P_H

#include <string>
#include "zmqprotobufnode.h"
#include <upns/services.pb.h>
#include <upns/services_internal.pb.h>

namespace upns {

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
    void handleRequestGenericEntry(upns::RequestGenericEntry *msg);
    void handleRequestStoreEntity(RequestStoreEntity* msg);
    void handleRequestStoreTree(upns::RequestStoreTree *msg);
//    void handleRequestStoreGenericEntry(upns::RequestStoreGenericEntry *msg);
//    void handleRequestEntity(upns::RequestEntity *msg);
//    void handleRequestTree(upns::RequestTree *msg);

    Repository *m_repo;
    std::string m_urlOutgoing;
    int m_portIncoming;

    // Implementation can be in cpp, because it's only used there. Must be public for function pointers
    template < typename T, void (upns::ZmqResponderPrivate::*func)(T*) >
    void toDelegate(google::protobuf::Message* msg);

private:
    friend class ZmqResponder;
};

}

#endif // UPNSZMQRESPONDER_P_H
