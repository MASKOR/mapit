#ifndef UPNSZMQRESPONDER_H
#define UPNSZMQRESPONDER_H

#include <string>
#include "repositoryserver.h"

namespace upns {

class ZmqResponderPrivate;
class Repository;

///
/// \brief The upns::ZmqResponder class
/// Acts as a server and tries to answer requests to a map-repository.
/// If a request can not be answered by this repository, the request may be forwarded to another node.
///

class ZmqResponder : public RepositoryServer
{
public:
    ZmqResponder( int portIncomingRequests, Repository* repo, std::string urlOutgoingRequests = std::string() );
    ~ZmqResponder();
private:
    ZmqResponderPrivate *m_d;

    // RepositoryServer interface
public:
    void handleRequest(int milliseconds);
};

}

#endif // UPNSZMQRESPONDER_H
