#include "upnszmqrequester_p.h"
#include "services.pb.h"
#include <functional>

template < typename T, void (upns::ZmqRequesterPrivate::*func)(T*) >
void upns::ZmqRequesterPrivate::toDelegate(google::protobuf::Message* msg)
{
    (this->*func)(static_cast<T*>(msg));
}

upns::ZmqRequesterPrivate::ZmqRequesterPrivate(std::__cxx11::string urlOutgoingRequests)
    :ZmqNode( false )
{
    void (upns::ZmqRequesterPrivate::*member)(google::protobuf::Message*);
    std::function<void(google::protobuf::Message*)> fn;

    member = &upns::ZmqRequesterPrivate::toDelegate<ReplyCheckoutFromMaster, &upns::ZmqRequesterPrivate::handleResponseCheckoutFromMaster>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<ReplyCheckoutFromMaster>( fn );

    member = &upns::ZmqRequesterPrivate::toDelegate<ReplyEntitydata, &upns::ZmqRequesterPrivate::handleResponseEntitydata>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<ReplyEntitydata>( fn );

    member = &upns::ZmqRequesterPrivate::toDelegate<ReplyHierarchy, &upns::ZmqRequesterPrivate::handleResponseHierarchy>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<ReplyHierarchy>( fn );

    member = &upns::ZmqRequesterPrivate::toDelegate<ReplyListCheckouts, &upns::ZmqRequesterPrivate::handleResponseListCheckouts>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<ReplyListCheckouts>( fn );

    member = &upns::ZmqRequesterPrivate::toDelegate<ReplyOperatorExecution, &upns::ZmqRequesterPrivate::handleResponseOperatorExecution>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<ReplyOperatorExecution>( fn );

    member = &upns::ZmqRequesterPrivate::toDelegate<ReplyStoreEntity, &upns::ZmqRequesterPrivate::handleResponseStoreEntity>;
    fn = std::bind(member, this, std::placeholders::_1);
    add_receivable_message_type<ReplyStoreEntity>( fn );
}
