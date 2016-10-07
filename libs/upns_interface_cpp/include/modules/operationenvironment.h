#ifndef OPERATIONENVIRONMENT_H
#define OPERATIONENVIRONMENT_H

#include "upns.h"

namespace upns
{
class CheckoutRaw;
class OperationDescription;
class OperationParameter;

class OperationEnvironment
{
public:
    /**
     * @brief mapServiceVersioned
     * This might be able to do a snapshot before the operation. afterwards it can see, what the operation did change.
     * @return do not delete this checkout!
     */
    virtual CheckoutRaw *getCheckout() const = 0;
    virtual const OperationDescription *getDescription() const = 0;
    virtual const std::string& getParameters() const = 0;
    virtual void setOutputDescription(const std::string&) = 0;
    virtual const OperationDescription outputDescription() const = 0;
};

}
#endif
