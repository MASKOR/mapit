#ifndef CHECKOUTIMPL_H
#define CHECKOUTIMPL_H

#include "upns_globals.h"
#include "services.pb.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include "serialization/abstractmapserializer.h"
#include "entitydata.h"
#include "versioning/checkout.h"
#include "modules/versioning/checkoutraw.h"
#include "util.h"
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
    Path preparePath(const Path &path);
    template <typename T>
    StatusCode createPath(const Path &path, upnsSharedPointer<T> createLeaf = upnsSharedPointer<T>(nullptr));
    template <typename T>
    StatusCode createObject(upnsSharedPointer<T> leafObject);
    template <typename T>
    StatusCode storeObject(upnsSharedPointer<T> leafObject);

    bool forEachPathSegment(const Path &path,
                                  std::function<bool(upnsString, size_t, bool)> before, std::function<bool(upnsString, size_t, bool)> after, const int start = 0);

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
    // TODO: go thru this method. heart of the system.
    // List of transientoidstoorigin is not correctly updated
    StatusCode s;
    Path p = preparePath(path);
    upnsVec<upnsPair<upnsSharedPointer<Tree>, bool> > exclusiveTreePath;
    ObjectId oid;
    upnsSharedPointer<Tree> current;
    bool rootMissing = m_checkout->rollingcommit().root().empty();
    bool rootNotExclusive = m_checkout->transientoidstoorigin().count(oid) == 0;
    if(rootMissing || rootNotExclusive)
    {
        // create/copy root for checkout
        ::std::string origin;
        if(rootMissing)
        {
            // create new root
            current = upnsSharedPointer<Tree>(new Tree);
            origin = "";
        }
        else
        {
            // copy to make exclusive
            ObjectId rootOid(m_checkout->rollingcommit().root());
            current = m_serializer->getTree(rootOid);
            origin = rootOid;
        }
        ::std::string toid = transientOid("");
        current->set_id(toid);
        m_checkout->mutable_rollingcommit()->set_root(toid);

        m_checkout->mutable_transientoidstoorigin()
                ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(m_checkout->rollingcommit().root(), origin));
        oid = toid;
        exclusiveTreePath.push_back(upnsPair<upnsSharedPointer<Tree>, bool>(current, true));
    }
    else
    {
        oid = m_checkout->rollingcommit().root();
        current = m_serializer->getTree(oid);
        exclusiveTreePath.push_back(upnsPair<upnsSharedPointer<Tree>, bool>(current, false));
    }

    bool leafWasStored = false;
    forEachPathSegment(p,
    [&](upnsString seg, size_t idx, bool isLast)
    {
        if(current == NULL) return false; // can not go futher
        if(seg.empty()) return false; // "//" not allowed
        oid = oidForChild(current, seg);
        ObjectId nextOid = transientOid(seg);
        // 6 cases:
        // 1+2) no oid yet          -> create tree/entity, put in vector to update and store
        // 3+4) exclusive oids      -> put in vector to update and store (for entity: create directly)
        // 5+6) non-exclusive oids  -> put copy in vector to update and store (for entity: create directly)
        if(oid.empty())
        {
            // create new
            if(!isLast || (isLast && createLeaf == NULL))
            {
                // append tree
                upnsSharedPointer<Tree> tree(new Tree);
                tree->set_id(nextOid);
                exclusiveTreePath.push_back(upnsPair<upnsSharedPointer<Tree>, bool>(tree, true));
            }
            else
            {
                // create Leaf. This is executed once. Next step is after(...).
                createLeaf->set_id(nextOid);
                s = createObject(createLeaf);
                if(upnsIsOk(s)) return false;
                leafWasStored = true;
            }
        }
        else
        {
            // use or copy existing
            bool segExclusive = m_checkout->transientoidstoorigin().count(oid) != 0;
            if(segExclusive)
            {
                // use existing
                if(!isLast || (isLast && createLeaf == NULL))
                {
                    // put tree in vector and do nothing
                    upnsSharedPointer<Tree> tree(m_serializer->getTree(oid));
                    exclusiveTreePath.push_back(upnsPair<upnsSharedPointer<Tree>, bool>(tree, false));
                }
                else
                {
                    // overwrite Leaf. This is executed once. Next step is after(...).
                    createLeaf->set_id(nextOid);
                    s = storeObject(createLeaf);
                    if(upnsIsOk(s)) return false;
                    leafWasStored = true;
                }
            }
            else
            {
                // copy
                if(!isLast || (isLast && createLeaf == NULL))
                {
                    // copy existing tree under transient oid
                    ObjectId nextOid = transientOid(seg);
                    upnsSharedPointer<Tree> tree(m_serializer->getTree(oid));
                    assert(tree);
                    tree->set_id(nextOid);
                    exclusiveTreePath.push_back(upnsPair<upnsSharedPointer<Tree>, bool>(tree, true));
                }
                else
                {
                    // overwrite Leaf with copy. This is executed once. Next step is after(...).
                    createLeaf->set_id(nextOid);
                    s = createObject(createLeaf);
                    if(upnsIsOk(s)) return false;
                    leafWasStored = true;
                }
            }

        }
        current = m_serializer->getTree(oid);
        return true; // continue thru path
    },
    [&](upnsString seg, size_t idx, bool isLast)
    {
        if(leafWasStored && isLast) return true; // leaf already created
        upnsPair<upnsSharedPointer<Tree>, bool> obj = exclusiveTreePath.back();
        exclusiveTreePath.pop_back();
        current = obj.first;
        if(obj.second)
        {
            s = m_serializer->createTree(current);
        }
        else
        {
            s = m_serializer->storeTree(current);
        }
        if(upnsIsOk(s)) return false; // must never happen. leads to inconsistent data. TODO: rollback
        if(exclusiveTreePath.empty())
        {
            assert(idx == 0);
            return false;
        }
        upnsSharedPointer<Tree> parent = exclusiveTreePath.back().first;
        ObjectReference oref;
        oref.set_id(current->id());
        oref.set_lastchange(QDateTime::currentDateTime().toMSecsSinceEpoch());
        assert(parent);
        parent->mutable_refs()
                ->insert( ::google::protobuf::MapPair< ::std::string, ::upns::ObjectReference>( seg, oref));
        log_info("dbg: Added obj " + p
                 + "name: " + seg
                 + "id: " + current->id());
        return true;
    });

    // update checkout commit
    assert(!rootMissing || rootMissing && m_checkout->transientoidstoorigin().count(current->id()) == 0);
    s = m_serializer->storeCheckoutCommit(m_checkout, m_name);
    return s;
}

}
#endif
