#ifndef OPERATIONENVIRONMENT_H
#define OPERATIONENVIRONMENT_H

#include <string>

namespace mapit
{
  namespace msgs {
    class OperationDescription;
    class OperationParameter;
  }
}

namespace upns
{
class CheckoutRaw;

class OperationEnvironment
{
public:
    /**
     * @brief mapServiceVersioned
     * This might be able to do a snapshot before the operation. afterwards it can see, what the operation did change.
     * @return do not delete this checkout!
     */
    virtual CheckoutRaw *getCheckout() const = 0;
    virtual const mapit::msgs::OperationDescription *getDescription() const = 0;
    virtual const std::string& getParameters() const = 0;
    virtual void setOutputDescription(const std::string&) = 0;
    virtual const mapit::msgs::OperationDescription outputDescription() const = 0;
};

}
#endif
