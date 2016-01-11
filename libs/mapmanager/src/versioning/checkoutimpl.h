#ifndef CHECKOUTIMPL_H
#define CHECKOUTIMPL_H

#include "upns_globals.h"
#include "services.pb.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include "modules/serialization/abstractmapserializerNEW.h"
#include "entitydata.h"
#include "versioning/checkout.h"
#include "modules/versioning/checkoutraw.h"

namespace upns
{

/**
 * @brief The CheckoutImpl class
 * Why?
 *
 * Checkout vs. CheckoutRaw
 * The Class "Checkout" is meant to be used by user/application. The Class "CheckoutRaw" is meant to be used by operators.
 * CheckoutRaw can edit objects and can not execute other operators (thus, recursive execution of operators is forbidden, it could break metadata).
 * Checkout can not edit objects, as the application can only do changes by executing operations!
 *
 * CheckoutCommon:
 * Both classes have similarities (e.g. all the read operations). These similarities are not repeated in both interfaces, but in "CheckoutCommon".
 *
 * CheckoutImpl:
 * When a Checkout is generated and given to an operator, the objects should not be copied. Thus, both of the classes ("Checkout" and "CheckoutRaw")
 * need to be one object with both implementations. The Implementation is in "CheckoutImpl". Instances of "CheckoutImpl" can be seen as "Chechout" for
 * the user/application and as "CheckoutRaw" fom an operator.
 *
 */
class CheckoutImpl : public Checkout, public CheckoutRaw /*, CheckoutCommon*/
{
public:
    /**
     * @brief Checkout Checkouts represent an editable state of a group of maps.
     * @param serializer
     * @param commitOrCheckoutId
     */
    CheckoutImpl(AbstractMapSerializer *serializer, const CommitId commitOrCheckoutId);
    CheckoutImpl(AbstractMapSerializer *serializer, const upnsSharedPointer<Branch> &branch);
    ~CheckoutImpl();

    virtual bool isInConflictMode();
    virtual upnsVec< upnsSharedPointer<Conflict> > getPendingConflicts();
    virtual upnsSharedPointer<Tree> getRoot();
    virtual upnsSharedPointer<Tree> getChild(ObjectId objectId);
    virtual OperationResult doOperation(const OperationDescription &desc);

    virtual upnsSharedPointer<AbstractEntityData> getEntityDataReadOnly(const ObjectId &entityId);
    virtual upnsSharedPointer<AbstractEntityData> getEntityDataForWrite(const ObjectId &entityId);

    virtual StatusCode storeTree(Path path, Tree tree);
    virtual StatusCode createTree(Path path, Tree tree);
    virtual upnsSharedPointer<Entity> createEntity(Path path, ObjectId parent);
    virtual void setConflictSolved(ObjectId solved);
protected:
    /**
     * @brief getEntityData Retrieves a data of the entity, which can be casted to a concrete type
     * After the internally used stream provider calls "endWrite()", the stream gets hashed and new ObjectIds are generated.
     * Entity::id -> hash of stream
     * Tree (layer) id -> child updated, rehash
     * Tree (map  ) id -> child updated, rehash
     * Tree (root ) id -> child updated, rehash
     * @param entityId
     * @return
     */
    upnsSharedPointer<AbstractEntityData> getEntityDataImpl(const ObjectId &entityId, bool readOnly);

private:
    upnsSharedPointer<AbstractEntityData> wrapEntityOfType(LayerType type,
                                                         upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider);
    upnsSharedPointer<AbstractEntityData> wrapEntityOfType(upnsString layertypeName,
                                                         upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider);
    AbstractMapSerializer* m_serializer;

    // Rolling Commit, id is random and not yet the hash of commit. It has the commit, this checkout is based on as "parents"
    CommitId m_checkoutId;
    // Branch, the checkout is based on, if any
    upnsSharedPointer<Branch> m_branch;
};

}
#endif
