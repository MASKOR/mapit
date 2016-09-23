#ifndef UPNSZMQREQUESTER_P_H
#define UPNSZMQREQUESTER_P_H

#include <string>
#include "zmqnode.h"
#include "services.pb.h"

namespace upns {

class ZmqRequesterPrivate : public ZmqNode
{

public:
    ZmqRequesterPrivate( std::string urlOutgoingRequests = std::string() );
    void handleResponseCheckoutFromMaster(ReplyCheckoutFromMaster* msg);
    void handleResponseEntitydata(ReplyEntitydata* msg);
    void handleResponseHierarchy(ReplyHierarchy* msg);
    void handleResponseListCheckouts(ReplyListCheckouts* msg);
    void handleResponseOperatorExecution(ReplyOperatorExecution* msg);
    void handleResponseStoreEntity(ReplyStoreEntity* msg);

    // Implementation can be in cpp, because it's only used there. Must be public for function pointers
    template < typename T, void (upns::ZmqRequesterPrivate::*func)(T*) >
    void toDelegate(google::protobuf::Message* msg);

private:
    friend class UpnsZmqNode;

//    template <typename T>
//    typedef void (*DelegateFunc)(T*);

};

} // namespace upns
#endif // UPNSZMQNODE_H
