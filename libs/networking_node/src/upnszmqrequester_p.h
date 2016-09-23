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

private:
    friend class UpnsZmqNode;
};

} // namespace upns
#endif // UPNSZMQNODE_H
