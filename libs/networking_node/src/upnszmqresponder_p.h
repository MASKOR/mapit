
#ifndef UPNSZMQRESPONDER_P_H
#define UPNSZMQRESPONDER_P_H

#include <string>

namespace upns {

class ZmqResponderPrivate
{

public:
    ZmqResponderPrivate( int portIncomingRequests, std::string urlOutgoingRequests = std::string() );

private:
    friend class ZmqResponder;
};

}

#endif // UPNSZMQRESPONDER_P_H
