#ifndef UPNSZMQREQUESTER_P_H
#define UPNSZMQREQUESTER_P_H

#include <string>
#include "zmqprotobufnode.h"
#include <upns/services.pb.h>
#include <upns/versioning/repository.h>

namespace upns {

class ZmqRequesterPrivate : public ZmqProtobufNode
{

public:
    ZmqRequesterPrivate( Repository* cache, std::string urlOutgoingRequests = std::string(), bool operationsLocal = false );
    Repository* m_cache;
    bool m_operationsLocal;
private:
    friend class UpnsZmqNode;
};

} // namespace upns
#endif // UPNSZMQREQUESTER_P_H
