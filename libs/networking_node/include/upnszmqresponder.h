
#ifndef UPNSZMQRESPONDER_H
#define UPNSZMQRESPONDER_H

#include <string>

namespace upns {

class ZmqResponderPrivate;

///
/// \brief The UpnsZmqNode class
/// Acts as a server and tries to answer requests to a map-repository.
/// If a request can not be answered by this repository, the request may be forwarded to another node.
///

class ZmqResponder
{
public:
    ZmqResponder( int portIncomingRequests, std::string urlOutgoingRequests = std::string() );
    ~ZmqResponder();
    int run();
private:
    ZmqResponderPrivate *m_d;
};

}

#endif // UPNSZMQRESPONDER_H
