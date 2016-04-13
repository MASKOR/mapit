#ifndef CHECKOUTIMPL_H
#define CHECKOUTIMPL_H

#include "upns_globals.h"
#include "services.pb.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include "serialization/abstractmapserializer.h"
#include "entitydata.h"
#include "versioning/checkout.h"
#include "modules/versioning/checkoutraw.h"
#include <functional>
#include <QDateTime>

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
    CheckoutImpl(AbstractMapSerializer *serializer, upnsSharedPointer<CheckoutObj> checkoutCommit, upnsString name, const upnsString branchname = NULL);
    ~CheckoutImpl();

    virtual bool isInConflictMode();
    virtual upnsVec< upnsSharedPointer<Conflict> > getPendingConflicts();
    virtual upnsSharedPointer<Tree> getRoot();
    virtual upnsSharedPointer<Tree> getTree(const Path &path);
    virtual upnsSharedPointer<Entity> getEntity(const Path &path);
    virtual upnsSharedPointer<Tree> getTreeConflict(const ObjectId &objectId);
    virtual upnsSharedPointer<Entity> getEntityConflict(const ObjectId &objectId);
    virtual OperationResult doOperation(const OperationDescription &desc);

    virtual upnsSharedPointer<AbstractEntityData> getEntityDataReadOnly(const Path &path);
    virtual upnsSharedPointer<AbstractEntityData> getEntityDataReadOnlyConflict(const ObjectId &entityId);
    virtual upnsSharedPointer<AbstractEntityData> getEntityDataForReadWrite(const Path &path);

    virtual StatusCode storeTree(const Path &path, upnsSharedPointer<Tree> tree);
    virtual StatusCode storeEntity(const Path &path, upnsSharedPointer<Entity> entity);

    virtual void setConflictSolved(const Path &path, const ObjectId &oid);

    template <typename T>
    inline upnsString generateTransientOid(const upnsSharedPointer<T> &obj);
private:

    ObjectId oidForChild(upnsSharedPointer<Tree> tree, const std::string &name);
    ObjectId oidForPath(const Path &path);
    template <typename T>
    StatusCode createPath(const Path &path, upnsSharedPointer<T> createLeaf = upnsSharedPointer<T>(nullptr));
    template <typename T>
    StatusCode createLeafObject(upnsSharedPointer<T> leafObject);

    StatusCode forEachPathSegment(const Path &path,
                                  std::function<bool(upnsString)> before, std::function<bool(upnsString)> after);

    /**
     * Depth first search for Commit, Tree and Entity.
     * Does not work for branches. Does not visit EntityData (must be done manually).
     * If before return false, after will not be executed.
     */
    StatusCode depthFirstSearch(upnsSharedPointer<Commit> obj,
                                std::function<bool(upnsSharedPointer<Commit>)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>)> afterCommit,
                                std::function<bool(upnsSharedPointer<Tree>)> beforeTree, std::function<bool(upnsSharedPointer<Tree>)> afterTree,
                                std::function<bool(upnsSharedPointer<Entity>)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>)> afterEntity);

    StatusCode depthFirstSearch(upnsSharedPointer<Tree> obj,
                                std::function<bool(upnsSharedPointer<Commit>)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>)> afterCommit,
                                std::function<bool(upnsSharedPointer<Tree>)> beforeTree, std::function<bool(upnsSharedPointer<Tree>)> afterTree,
                                std::function<bool(upnsSharedPointer<Entity>)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>)> afterEntity);

    StatusCode depthFirstSearch(upnsSharedPointer<Entity> obj,
                                std::function<bool(upnsSharedPointer<Commit>)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>)> afterCommit,
                                std::function<bool(upnsSharedPointer<Tree>)> beforeTree, std::function<bool(upnsSharedPointer<Tree>)> afterTree,
                                std::function<bool(upnsSharedPointer<Entity>)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>)> afterEntity);

    inline upnsString transientOid(const upnsString &path);
    AbstractMapSerializer* m_serializer;

    // Rolling Commit, id is random and not yet the hash of commit. This commit is exclusive for this checkout, this checkout is based on as "parents"
    // TODO: maybe leave id out in every object
    //upnsSharedPointer<Commit>  m_commit;

    // Branch, the checkout is based on, if any
    upnsString m_branchname;

    // Name of the checkout
    upnsString m_name;

    upnsSharedPointer<CheckoutObj> m_checkout;

    upnsVec<ObjectId> m_transientOids;
    ObjectId m_nextTransientOid;
};

template <typename T>
StatusCode CheckoutImpl::createPath(const Path &path, upnsSharedPointer<T> createLeaf)
{
    // path p has no beginning / and always trailing /
    Path p;
    if(path[0] == '/')
    {
        p = path.substr(1);
    }
    else
    {
        p = path;
    }
    if(p.length() != 0 && p[p.length()-1] != '/')
    {
        p += "/";
    }
    if(m_checkout->rollingcommit().root().empty())
    {
        ::std::string *rootOid = m_checkout->mutable_rollingcommit()->mutable_root();
        rootOid->append(transientOid(""));
        StatusCode s = m_serializer->storeCheckoutCommit(m_checkout, m_name);
        if(!upnsIsOk(s)) return s;
    }
    ObjectId oid(m_checkout->rollingcommit().root());
    upnsSharedPointer<Tree> current;
    upnsSharedPointer<Tree> parent;
    size_t lastSlash = 0;
    while(true)
    {
        size_t nextSlash = p.find_first_of('/', lastSlash);
        assert(nextSlash != std::string::npos);
        if(nextSlash == p.length()-1) // || string::npos
        {
            // last slash found
            return UPNS_STATUS_OK;
        }
        size_t nextNextSlash = p.find_first_of('/', nextSlash);
        parent = current;
        current = m_serializer->getTree(oid);
        bool isLeaf = (nextNextSlash == p.length()-1);
        if(isLeaf && createLeaf)
        {
            // Create Entity/Tree
            ObjectId nextOid = transientOid(p.substr(0, nextSlash));
            createLeaf->set_id(nextOid);
            StatusCode s = createLeafObject(createLeaf);
            if(upnsIsOk(s))
            {
                ObjectReference oref;
                oref.set_id(nextOid);
                oref.set_lastchange(QDateTime::currentDateTime().toMSecsSinceEpoch());
                ::std::string nam = p.substr(lastSlash, nextSlash);
                assert(current);
                current->mutable_refs()->insert( ::google::protobuf::MapPair< ::std::string, ::upns::ObjectReference>( nam, oref));
                s = m_serializer->storeTree(current);
                log_info("Added obj " + p
                         + "name: " + p.substr(lastSlash, nextSlash)
                         + "id: " + nextOid);
                return s;
            }
            else
            {
                log_error( "Error occured while creating path: " + p + "name:" + p.substr(lastSlash, nextSlash));
                return s;
            }
        }
        if(current == NULL)
        {
            upnsSharedPointer<Entity> currentEnt = m_serializer->getEntity(oid);
            if(currentEnt != NULL)
            {
                // if the path is not yet finished and there is a non tree
                log_error("Element not found or non-Tree was part of path: " + p + " (" + p.substr(lastSlash, nextSlash) + ") Oid: " + oid);
                return UPNS_STATUS_INVALID_ARGUMENT;
            }
            else
            {
                ObjectId nextOid = transientOid(p.substr(0, nextSlash));
                StatusCode s;
                if(createLeaf)
                {
                    // Create Entity
                    upnsSharedPointer<Tree> tree(new Tree);
                    tree->set_id(nextOid);
                    s = m_serializer->createTree(tree);
                }
                if(upnsIsOk(s))
                {
                    // update parent
                    ObjectReference oref;
                    oref.set_id(nextOid);
                    oref.set_lastchange(QDateTime::currentDateTime().toMSecsSinceEpoch());
                    ::std::string nam = p.substr(lastSlash, nextSlash);
                    assert(parent);
                    parent->mutable_refs()->insert( ::google::protobuf::MapPair< ::std::string, ::upns::ObjectReference>( nam, oref));
                    s = m_serializer->storeTree(parent);
                    log_info("Added obj " + p
                             + "name: " + p.substr(lastSlash, nextSlash)
                             + "id: " + nextOid);
                }
                else
                {
                    log_error( "Error occured while creating path: " + p + "name:" + p.substr(lastSlash, nextSlash));
                    return s;
                }
            }
        }
        oid = oidForChild(current, p.substr(lastSlash, nextSlash));
        assert(nextSlash+1 <= p.length()-1);
        lastSlash = nextSlash+1;
    }
}

}
#endif
