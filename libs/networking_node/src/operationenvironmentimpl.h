#ifndef __OPERATIONENVIRONMENTIMPL_H
#define __OPERATIONENVIRONMENTIMPL_H
#include <upns/operators/operationenvironment.h>
#include <mapit/msgs/services.pb.h>

namespace upns
{
class CheckoutRaw;

class OperationEnvironmentImpl : public OperationEnvironment
{
public:
    OperationEnvironmentImpl(const OperationDescription& desc);
    void setCheckout(CheckoutRaw *checkout);

    // OperationEnvironment Interface
    virtual CheckoutRaw *getCheckout() const;
    virtual const OperationDescription *getDescription() const;
    virtual const std::string& getParameters() const;
    virtual void setOutputDescription(const std::string& out);
    virtual const OperationDescription outputDescription() const;

private:
    CheckoutRaw *m_checkout;
    const OperationDescription m_operationDesc;
    OperationDescription m_outDesc;
};

}

#endif
