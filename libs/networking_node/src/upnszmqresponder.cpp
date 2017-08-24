#include "upnszmqresponder.h"
#include "upnszmqresponder_p.h"
#include <upns/versioning/repository.h>
#include "upns/repositoryserver.h"
#include <zmq.hpp>

upns::ZmqResponder::ZmqResponder(int portIncomingRequests, Repository* repo, std::string urlOutgoingRequests)
{
    m_d = new upns::ZmqResponderPrivate(portIncomingRequests, repo, urlOutgoingRequests);
}

upns::ZmqResponder::~ZmqResponder()
{
    delete m_d;
}

void upns::ZmqResponder::handleRequest(int milliseconds)
{
    m_d->receive_and_dispatch(milliseconds);
}

