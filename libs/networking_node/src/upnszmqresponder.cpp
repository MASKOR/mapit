#include "upnszmqresponder.h"
#include "upnszmqresponder_p.h"
#include <zmq.hpp>

UpnsZmqResponder::UpnsZmqResponder(int portIncomingRequests, std::__cxx11::string urlOutgoingRequests)
{
    zmq::socket_t parentRequester(context, ZMQ_REQ);
    parentRequester.connect( url );
    m_d = new UpnsZmqResponderPrivate();
}

UpnsZmqResponder::~UpnsZmqResponder()
{
    delete m_d;
}

int UpnsZmqResponder::run()
{
    while (true) {
        zmq::message_t request;

        //  Wait for next request from client
        socket.recv (&request);
        std::cout << "Received Hello" << std::endl;

        //  Do some 'work'
        sleep(1);

        //  Send reply back to client
        zmq::message_t reply (5);
        memcpy (reply.data (), "World", 5);
        socket.send (reply);
    }
}
