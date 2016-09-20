#include "upnszmqrequester.h"
#include "upnszmqrequester_p.h"
#include <zmq.hpp>

upns::ZmqRequester::ZmqRequester(std::__cxx11::string urlOutgoingRequests)
    :m_d( new upns::ZmqRequesterPrivate(urlOutgoingRequests) )
{
}

upns::ZmqRequester::~ZmqRequester()
{
    delete m_d;
}
