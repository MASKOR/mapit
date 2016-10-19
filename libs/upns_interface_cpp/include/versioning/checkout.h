#ifndef CHECKOUT_H
#define CHECKOUT_H

#include "upns_globals.h"
#include "services.pb.h"
#include "checkoutcommon.h"

namespace upns
{
class OperationEnvironment;

/**
 * @brief A Checkout object represents an editable state/version of all maps.
 * Checkout (without raw) is the interface for users to invoke parameterized Operators to indirectly edit objects.
 * Changes can be done in a checkout without creating new commits. Checkouts are persistent on harddrive and do
 * not only exist in RAM (just like a git working directory persists on harddrive). When they are commited, the
 * checkout gets transformed to the final commit. At this stage, it can not be changed anymore (all objects are
 * immutable). Objects (entities, trees) may not have an objectId in a checkout. One can not use the commit- and
 * objectIds, because these are changing. Ids are generated during a commit. Some objects, which have not changed
 * since the last commit, may have an objectId. These objects may be copied on the first write access. However if
 * the history of the objects is traceable and the object can bes restored from other data, the system may over-
 * write the data without copying. OperationDescriptors are collected while the checkout is active. Operations
 * inside the list can have parameters (objectIds) which are only temporary in the system. When replayed,
 * dependencymanagement tries to compute all Operations which have all their parameters ready.
 */
class Checkout : public CheckoutCommon
{
public:
    virtual ~Checkout() {}
    /**
     * @brief doOperation Executes the Operator given in the description
     * @param desc
     * @return
     */
    virtual OperationResult doOperation(const OperationDescription &desc) = 0;

    /**
     * @brief doUntraceableOperation Executes a custom operation.
     * This is meant to get raw sensor data into the system.
     * Whenever possible, doOperation should be used. Using this method extensively will
     * result in huge amounts of data and large repositories.
     * @param desc
     * @return
     */
    virtual OperationResult doUntraceableOperation(const OperationDescription &desc, std::function<upns::StatusCode(upns::OperationEnvironment*)> operate) = 0;
};

}
#endif
