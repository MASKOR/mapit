#include "operationenvironmentimpl.h"
#include "versioning/checkoutraw.h"
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

const OperationParameter *OperationEnvironmentImpl::getParameter(const std::string &key) const
{
    for(int p=0; p < m_operationDesc.params_size() ; ++p)
    {
        const OperationParameter *param = &m_operationDesc.params(p);
        if(param->key() == key)
        {
            return param;
        }
    }
    return NULL;
}

void OperationEnvironmentImpl::setOutputDescription(OperationDescription out)
{
    m_outDesc = out;
}

const OperationDescription &OperationEnvironmentImpl::outputDescription() const
{
    return m_outDesc;
}

}
