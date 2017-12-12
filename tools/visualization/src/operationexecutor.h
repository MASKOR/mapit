#ifndef OPERATIONEXECUTOR_H
#define OPERATIONEXECUTOR_H

#include <QThread>
#include <memory>
#include <mapit/msgs/datastructs.pb.h>
#include <upns/typedefs.h>

namespace upns
{
    class Checkout;
}

class OperationExecutor : public QThread
{
    Q_OBJECT
    void run();
public:
    OperationExecutor(QObject * parent, std::shared_ptr<upns::Checkout> co, mapit::msgs::OperationDescription desc);
    ~OperationExecutor();
Q_SIGNALS:
    void operationExecuted( int status );

private:
    std::shared_ptr<upns::Checkout> m_checkout;
    mapit::msgs::OperationDescription m_desc;
};
#endif
