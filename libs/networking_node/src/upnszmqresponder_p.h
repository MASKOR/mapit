#ifndef UPNSZMQRESPONDER_P_H
#define UPNSZMQRESPONDER_P_H

#include <string>
#include "zmqprotobufnode.h"
#include "services.pb.h"
#include "services_internal.pb.h"

namespace upns {

class Repository;

class ZmqResponderPrivate : public ZmqProtobufNode
{

public:
    ZmqResponderPrivate( int portIncomingRequests, Repository* repo, std::string urlOutgoingRequests = std::string() );
    void handleRequestCheckout(RequestCheckout* msg);
    void handleRequestEntitydata(RequestEntitydata* msg);
    void handleRequestHierarchy(RequestHierarchy* msg);
    void handleRequestListCheckouts(RequestListCheckouts* msg);
    void handleRequestOperatorExecution(RequestOperatorExecution* msg);
    void handleRequestStoreEntity(RequestStoreEntity* msg);
    void handleRequestEntity(upns::RequestEntity *msg);
    void handleRequestTree(upns::RequestTree *msg);

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
