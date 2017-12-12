#include "operationexecutor.h"
#include <upns/errorcodes.h>
#include <upns/versioning/checkout.h>

void OperationExecutor::run()
{
    if(m_checkout)
    {
        upns::OperationResult s = m_checkout->doOperation(m_desc);
        Q_EMIT operationExecuted( s.first );
    }
    else
    {
        Q_EMIT operationExecuted( UPNS_STATUS_ERROR );
    }
}

OperationExecutor::OperationExecutor(QObject *parent, std::shared_ptr<upns::Checkout> co, mapit::msgs::OperationDescription desc)
    : QThread(parent)
    , m_checkout( co )
    , m_desc( desc )
{}

OperationExecutor::~OperationExecutor() {wait();}
