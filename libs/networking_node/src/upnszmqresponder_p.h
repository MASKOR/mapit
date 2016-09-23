#ifndef UPNSZMQRESPONDER_P_H
#define UPNSZMQRESPONDER_P_H

#include <string>
#include "zmqnode.h"
#include "services.pb.h"

namespace upns {

class Repository;

class ZmqResponderPrivate : public ZmqNode
{

public:
    ZmqResponderPrivate( int portIncomingRequests, Repository* repo, std::__cxx11::string urlOutgoingRequests = std::__cxx11::string() );
    void handleRequestCheckoutFromMaster(RequestCheckoutFromMaster* msg);
    void handleRequestEntitydata(RequestEntitydata* msg);
    void handleRequestHierarchy(RequestHierarchy* msg);
    void handleRequestListCheckouts(RequestListCheckouts* msg);
    void handleRequestOperatorExecution(RequestOperatorExecution* msg);
    void handleRequestStoreEntity(RequestStoreEntity* msg);

    Repository *m_repo;
    std::__cxx11::string m_urlOutgoing;
    int m_portIncoming;

    // Implementation can be in cpp, because it's only used there. Must be public for function pointers
    template < typename T, void (upns::ZmqResponderPrivate::*func)(T*) >
    void toDelegate(google::protobuf::Message* msg);

private:
    friend class ZmqResponder;
};

}

#endif // UPNSZMQRESPONDER_P_H
