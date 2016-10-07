#include "operationenvironmentimpl.h"
#include "modules/versioning/checkoutraw.h"
#include "../include/modules/operationenvironment.h"

namespace upns
{

OperationEnvironmentImpl::OperationEnvironmentImpl(const OperationDescription &desc)
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

const OperationDescription *OperationEnvironmentImpl::getDescription() const
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

const OperationDescription OperationEnvironmentImpl::outputDescription() const
{
    return m_outDesc;
}

}
