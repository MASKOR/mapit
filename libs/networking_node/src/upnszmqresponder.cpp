#include "upnszmqresponder.h"
#include "upnszmqresponder_p.h"
#include "versioning/repository.h"
#include "repositoryserver.h"
#include <zmq.hpp>

upns::ZmqResponder::ZmqResponder(int portIncomingRequests, Repository* repo, std::__cxx11::string urlOutgoingRequests)
{
    m_d = new upns::ZmqResponderPrivate(portIncomingRequests, repo, urlOutgoingRequests);
}

upns::ZmqResponder::~ZmqResponder()
{
    delete m_d;
}

void upns::ZmqResponder::handleRequest()
{
    m_d->receive_and_dispatch();
}
