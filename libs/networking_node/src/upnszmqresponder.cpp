#include "upnszmqresponder.h"
#include "upnszmqresponder_p.h"
#include <zmq.hpp>

upns::ZmqResponder::ZmqResponder(int portIncomingRequests, std::__cxx11::string urlOutgoingRequests)
{
    m_d = new upns::ZmqResponderPrivate(portIncomingRequests, urlOutgoingRequests);
}

upns::ZmqResponder::~ZmqResponder()
{
    delete m_d;
}

int upns::ZmqResponder::run()
{
    while (true) {

    }
}
