#include "operationenvironmentimpl.h"
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>

namespace upns
{

OperationEnvironmentImpl::OperationEnvironmentImpl(const mapit::msgs::OperationDescription &desc)
    :m_operationDesc( desc )
{
}

void OperationEnvironmentImpl::setCheckout(CheckoutRaw *checkout)
{
    m_checkout = checkout;
}

CheckoutRaw *OperationEnvironmentImpl::getCheckout() const
{
    return m_checkout;
}

const mapit::msgs::OperationDescription *OperationEnvironmentImpl::getDescription() const
{
    return &m_operationDesc;
}

const std::string& OperationEnvironmentImpl::getParameters() const
{
    return m_operationDesc.params();
}

void OperationEnvironmentImpl::setOutputDescription(const std::string& out)
{
    m_outDesc.set_params(out);
}

const mapit::msgs::OperationDescription OperationEnvironmentImpl::outputDescription() const
{
    return m_outDesc;
}

}
