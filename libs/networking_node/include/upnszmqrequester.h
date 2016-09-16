
#ifndef UPNSZMQREQUESTER_H
#define UPNSZMQREQUESTER_H

#include <string>
#include "versioning/repository.h"

namespace upns {

class ZmqRequesterPrivate;

///
/// \brief The UpnsZmqRequester class
/// Implements the basic Repository Interface and will send requests over network
///

class ZmqRequester : public upns::Repository
{
public:
    ZmqRequester( int portIncomingRequests, std::string urlOutgoingRequests = std::string() );
    ~ZmqRequester();
private:
    ZmqRequesterPrivate *m_d;
};

}

#endif // UPNSZMQREQUESTER_H
