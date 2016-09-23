#include "upnszmqresponder.h"
#include "upnszmqresponder_p.h"
#include "versioning/repository.h"
#include "repositoryserver.h"
#include <zmq.hpp>

upns::ZmqResponder::ZmqResponder(int portIncomingRequests, Repository* repo, std::__cxx11::string urlOutgoingRequests)
{
    m_d = new upns::ZmqResponderPrivate(portIncomingRequests, repo, urlOutgoingRequests);
    m_d->bind("tcp://*:" + std::__cxx11::to_string( m_d->m_portIncoming ) );
}

upns::ZmqResponder::~ZmqResponder()
{
    delete m_d;
}

void upns::ZmqResponder::handleRequest()
{
    m_d->handle_receive();
}

