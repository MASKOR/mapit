#include "upnszmqresponder_p.h"
#include "services.pb.h"

upns::ZmqResponderPrivate::ZmqResponderPrivate(int portIncomingRequests, std::__cxx11::string urlOutgoingRequests)
{
    add_receivable_message_type<RequestCheckoutFromMaster>();
    add_receivable_message_type<RequestEntitydata>();
    add_receivable_message_type<RequestHierarchy>();
    add_receivable_message_type<RequestListCheckouts>();
    add_receivable_message_type<RequestOperatorExecution>();
    add_receivable_message_type<RequestStoreEntity>();
}
