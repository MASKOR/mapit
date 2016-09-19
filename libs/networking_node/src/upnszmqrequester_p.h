
#ifndef UPNSZMQREQUESTER_P_H
#define UPNSZMQREQUESTER_P_H

#include <string>
#include "zmqnode.h"

namespace upns {

class UpnsZmqRequesterPrivate : public ZMQNode
{

public:
    UpnsZmqRequesterPrivate( int portIncomingRequests, std::string urlOutgoingRequests = std::string() );

private:
    friend class UpnsZmqNode;
};

} // namespace upns
#endif // UPNSZMQNODE_H
