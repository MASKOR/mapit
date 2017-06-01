#ifndef __OPERATIONENVIRONMENTIMPL_H
#define __OPERATIONENVIRONMENTIMPL_H
#include <upns/operators/operationenvironment.h>
#include <mapit/msgs/services.pb.h>

namespace upns
{

class OperationEnvironmentImpl : public OperationEnvironment
{
public:
    OperationEnvironmentImpl(const mapit::msgs::OperationDescription& desc);
    void setCheckout(CheckoutRaw *checkout);

    // OperationEnvironment Interface
    virtual CheckoutRaw *getCheckout() const;
    virtual const mapit::msgs::OperationDescription *getDescription() const;
    virtual const std::string& getParameters() const;
    virtual void setOutputDescription(const std::string& out);
    virtual const mapit::msgs::OperationDescription outputDescription() const;

private:
    CheckoutRaw *m_checkout;
    const mapit::msgs::OperationDescription m_operationDesc;
    mapit::msgs::OperationDescription m_outDesc;
};

}

#endif
