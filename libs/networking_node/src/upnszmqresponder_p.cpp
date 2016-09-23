#include "upnszmqresponder_p.h"
#include "services.pb.h"
#include <functional>

template < typename T, void (upns::ZmqResponderPrivate::*func)(T*) >
void upns::ZmqResponderPrivate::toDelegate(google::protobuf::Message* msg)
{
    (this->*func)(static_cast<T*>(msg));
}

upns::ZmqResponderPrivate::ZmqResponderPrivate(int portIncomingRequests, Repository *repo, std::__cxx11::string urlOutgoingRequests)
    :ZmqNode( true ),
      m_repo( repo ),
      m_urlOutgoing( urlOutgoingRequests ),
      m_portIncoming( portIncomingRequests )
{
    void (upns::ZmqResponderPrivate::*member)(google::protobuf::Message*);
    std::function<void(google::protobuf::Message*)> fn;

    member = &upns::ZmqResponderPrivate::toDelegate<RequestCheckoutFromMaster, &upns::ZmqResponderPrivate::handleRequestCheckoutFromMaster>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestCheckoutFromMaster>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestEntitydata, &upns::ZmqResponderPrivate::handleRequestEntitydata>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestEntitydata>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestHierarchy, &upns::ZmqResponderPrivate::handleRequestHierarchy>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestHierarchy>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestListCheckouts, &upns::ZmqResponderPrivate::handleRequestListCheckouts>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestListCheckouts>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestOperatorExecution, &upns::ZmqResponderPrivate::handleRequestOperatorExecution>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestOperatorExecution>( fn );

    member = &upns::ZmqResponderPrivate::toDelegate<RequestStoreEntity, &upns::ZmqResponderPrivate::handleRequestStoreEntity>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<RequestStoreEntity>( fn );
}

void upns::ZmqResponderPrivate::handleRequestCheckoutFromMaster(RequestCheckoutFromMaster *msg)
{

}

void upns::ZmqResponderPrivate::handleRequestEntitydata(RequestEntitydata *msg)
{

}

void upns::ZmqResponderPrivate::handleRequestHierarchy(RequestHierarchy *msg)
{

}

void upns::ZmqResponderPrivate::handleRequestListCheckouts(RequestListCheckouts *msg)
{

}

void upns::ZmqResponderPrivate::handleRequestOperatorExecution(RequestOperatorExecution *msg)
{

}

void upns::ZmqResponderPrivate::handleRequestStoreEntity(RequestStoreEntity *msg)
{

}
