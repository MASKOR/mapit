#ifndef UPNSZMQREQUESTER_P_H
#define UPNSZMQREQUESTER_P_H

#include <string>
#include "zmqprotobufnode.h"
#include "services.pb.h"
#include "versioning/repository.h"

namespace upns {

class ZmqRequesterPrivate : public ZmqProtobufNode
{

public:
    ZmqRequesterPrivate( Repository* cache, std::string urlOutgoingRequests = std::string() );
    Repository* m_cache;
private:
    friend class UpnsZmqNode;
};

} // namespace upns
#endif // UPNSZMQREQUESTER_P_H
