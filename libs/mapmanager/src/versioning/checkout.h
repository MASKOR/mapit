#ifndef CHECKOUT_H
#define CHECKOUT_H

#include "upns_globals.h"
#include "services.pb.h"
#include "abstractentitydatastreamprovider.h"
#include "../serialization/abstractmapserializerNEW.h"
#include "entitydata.h"
#include "checkoutcommon.h"

namespace upns
{

/**
 * @brief A Checkout object represents an editable state/version of all maps.
 * Checkout (without raw) is the interface for User to invoke parameterized Operators to indirectly edit objects.
 * Changes can be done in a checkout without creating
 * new commits. Checkouts are serialized even before they are commited (just like a git working directory persists on harddrive).
 * When they are commited, the checkout gets transformed to the final commit. At this stage, it can not be changed anymore.
 * While not commited, checkouts behave like commits but one can not use the commit- and objectIds, because these are changing.
 * In contrast to all other places, objects behind the commit- and objectIds inside a checkout can be deleted from the system completely.
 * OperationDescriptors are collected while the checkout is active. Operations inside the list can have parameters (objectIds) which are
 * only temporary in the system. When replayed, dependencymanagement tries to compute all Operations which have all their parameters ready.
 */
class Checkout : public CheckoutCommon
{
public:
    /**
     * @brief doOperation Executes the Operator given in the description
     * @param desc
     * @return
     */
    virtual OperationResult doOperation(const OperationDescription &desc) = 0;

    /**
     * @brief getEntityData Retrieves binary stream of entitydata which can be casted to concrete type. This is always read only!
     * @param entityId
     * @return
     */
    virtual upnsSharedPointer<AbstractEntityData> getEntityData(const ObjectId &entityId) = 0;
};

}
#endif
