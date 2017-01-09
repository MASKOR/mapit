#include "upnszmqrequester_p.h"
#include "services.pb.h"
#include "services_internal.pb.h"
#include <functional>

upns::ZmqRequesterPrivate::ZmqRequesterPrivate(Repository *cache, std::string urlOutgoingRequests, bool operationsLocal)
    :ZmqProtobufNode( false ),
     m_cache( cache ),
     m_operationsLocal(operationsLocal)
{
    connect(urlOutgoingRequests);
    add_receivable_message_type<ReplyCheckout>();
    add_receivable_message_type<ReplyEntitydata>();
    add_receivable_message_type<ReplyHierarchy>();
    add_receivable_message_type<ReplyListCheckouts>();
    add_receivable_message_type<ReplyOperatorExecution>();
    add_receivable_message_type<ReplyGenericEntry>();
    add_receivable_message_type<ReplyStoreEntity>();
    add_receivable_message_type<ReplyStoreTree>();
}
