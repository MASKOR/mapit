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
    CheckoutImpl(AbstractMapSerializer *serializer, upnsSharedPointer<CheckoutObj> checkoutCommit, upnsString name, const upnsString branchname = "");
    virtual ~CheckoutImpl();

    virtual bool isInConflictMode();
    virtual upnsVec< upnsSharedPointer<Conflict> > getPendingConflicts();
    virtual upnsSharedPointer<Tree> getRoot();
    virtual upnsSharedPointer<Tree> getTree(const Path &path);
    virtual upnsSharedPointer<Entity> getEntity(const Path &path);
    virtual upnsSharedPointer<Tree> getTreeConflict(const ObjectId &objectId);
    virtual upnsSharedPointer<Entity> getEntityConflict(const ObjectId &objectId);
    virtual OperationResult doOperation(const OperationDescription &desc);

    virtual upnsSharedPointer<AbstractEntityData> getEntitydataReadOnly(const Path &path);
    virtual upnsSharedPointer<AbstractEntityData> getEntitydataReadOnlyConflict(const ObjectId &entityId);
    virtual upnsSharedPointer<AbstractEntityData> getEntityDataForReadWrite(const Path &path);

    virtual StatusCode storeTree(const Path &path, upnsSharedPointer<Tree> tree);
    virtual StatusCode storeEntity(const Path &path, upnsSharedPointer<Entity> entity);

    virtual void setConflictSolved(const Path &path, const ObjectId &oid);

    virtual upnsSharedPointer<Branch> getParentBranch();
    virtual upnsVec<CommitId> getParentCommitIds();

    template <typename T>
    inline upnsString generateTransientOid(const upnsSharedPointer<T> &obj);

    StatusCode depthFirstSearch(std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> afterCommit,
                                std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> afterTree,
                                std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> afterEntity);
    upnsSharedPointer<CheckoutObj> getCheckoutObj();
    const upnsString& getName() const;
private:

    /**
     * @brief oidForChild Used to convert path to oids.
     * @param tree parent
     * @param name of the child (path segment, without slashes)
     * @return Oid or empty oid
     */
    ObjectId oidForChild(upnsSharedPointer<Tree> tree, const std::string &name);

    /**
     * @brief oidForPath Used to convert path to oids.
     * @param path beginning with root dir of checkout. Checkout must not be part of path. Can have leading or trailing slashes.
     * @return Oid or empty oid
     */
    ObjectId oidForPath(const Path &path);

    // helper, used to ensure slashes at beginning and end
    Path preparePath(const Path &path);

    /**
     * @brief createPath If checkout wants to write to a path, it must be created. The leaf can be a tree or entity. This is not a trivial function, but should be easy to use from the outside.
     * @param path path to create in the checkout.
     * @param createLeaf null, if the leaf exists as non-exclusive tree/entity. Can also be a new entity. Can also be a tree (e.g. copy/move operation).
     */
    template <typename T>
    StatusCode createPath(const Path &path, upnsSharedPointer<T> createLeaf = upnsSharedPointer<T>(nullptr));

    /**
     * @brief Stores all kind of objects at a path.
     */
    template <typename T>
    upnsPair<StatusCode, ObjectId> storeObject(upnsSharedPointer<T> leafObject, const ObjectId &transId);

    bool forEachPathSegment(const Path &path,
                                  std::function<bool(upnsString, size_t, bool)> before, std::function<bool(upnsString, size_t, bool)> after, const int start = 0);

    /**
     * @brief Depth first search for Commit, Tree and Entity.
     * Does not work for branches. Does not visit EntityData (must be done manually).
     * If "before" returns false, "after" will not be executed.
     */
    StatusCode depthFirstSearch(upnsSharedPointer<Commit> obj, const ObjectId& oid, const Path &path,
                                std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> afterCommit,
                                std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> afterTree,
                                std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> afterEntity);

    StatusCode depthFirstSearch(upnsSharedPointer<Tree> obj, const ObjectId& oid, const Path &path,
                                std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> afterCommit,
                                std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> afterTree,
                                std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> afterEntity);

    StatusCode depthFirstSearch(upnsSharedPointer<Entity> obj, const ObjectId& oid, const Path &path,
                                std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectId&, const Path &)> afterCommit,
                                std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectId&, const Path &)> afterTree,
                                std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectId&, const Path &)> afterEntity);

    AbstractMapSerializer* m_serializer;

    // Branch, the checkout is based on, if any
    upnsString m_branchname;

    // Name of the checkout
    upnsString m_name;

    upnsSharedPointer<CheckoutObj> m_checkout;

    upnsVec<ObjectId> m_transientOids;
    ObjectId m_nextTransientOid;
};

//TODO: This method could be implemented shorter with less branching and duplication in code.
template <typename T>
StatusCode CheckoutImpl::createPath(const Path &path, upnsSharedPointer<T> createLeaf)
{
    Path p = preparePath(path);
    upnsVec< upnsSharedPointer<Tree> > exclusiveTreePath;
    upnsSharedPointer<Tree> current;
    bool rootMissing = m_checkout->rollingcommit().root().empty();
    bool rootNotExclusive = m_checkout->transientoidstoorigin().count(m_checkout->rollingcommit().root()) == 0;
    // if there is no root directory, checkout must be empty and have nothing transient
    assert(!rootMissing || rootMissing && m_checkout->transientoidstoorigin_size() == 0);
    if(rootMissing || rootNotExclusive)
    {
        // create/copy root for checkout
        ::std::string origin;
        if(rootMissing)
        {
            // create new root
            origin = "";
            current = upnsSharedPointer<Tree>(new Tree);
        }
        else
        {
            // copy to make exclusive
            origin = m_checkout->rollingcommit().root();
            current = m_serializer->getTree(origin);
        }
        m_checkout->mutable_rollingcommit()->set_root(m_name + "/");
        m_checkout->mutable_transientoidstoorigin()
                ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(m_checkout->rollingcommit().root(), origin));
        exclusiveTreePath.push_back( current );
    }
    else
    {
        // root exists and can exclusively be altered
        ObjectId oid = m_checkout->rollingcommit().root();
        current = m_serializer->getTreeTransient(oid);
        exclusiveTreePath.push_back( current );
    }

    bool leafWasStored = false;
    ObjectId leafOid;
    forEachPathSegment(p,
    [&](upnsString seg, size_t idx, bool isLast)
    {
        if(current == NULL) return false; // can not go futher
        if(seg.empty()) return false; // "//" not allowed
        ObjectId oid = oidForChild(current, seg);
        ObjectId transOid = m_name + "/" + p.substr(0,idx);
        // 6 cases:
        // 1+2) no oid yet          -> create tree/entity, put in vector to update and store. (for leaf: create directly)
        // 3+4) exclusive oids      -> put in vector to update and store (for leaf: create directly)
        // 5+6) non-exclusive oids  -> put copy in vector to update and store (for leaf: create directly)
        if(oid.empty())
        {
            // create new
            if(!isLast || (isLast && createLeaf == NULL))
            {
                // append tree
                upnsSharedPointer<Tree> tree(new Tree);
                exclusiveTreePath.push_back( tree );
                m_checkout->mutable_transientoidstoorigin()
                        ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(transOid, ""));
                current = tree;
            }
            else
            {
                // create Leaf. This is executed once. Next step is after(...).
                upnsPair<StatusCode, ObjectId> soid = storeObject(createLeaf, transOid);
                leafOid = soid.second;
                if(!upnsIsOk(soid.first)) return false;
                m_checkout->mutable_transientoidstoorigin()
                        ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(soid.second, "")); //TODO: history of createLeaf lost, if ther was one

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
                    upnsSharedPointer<Tree> tree(m_serializer->getTreeTransient(oid));
                    if(tree == NULL)
                    {
                        log_error("Segment of path was not a tree");
                        return false;
                    }
                    exclusiveTreePath.push_back( tree );
                    current = tree;
                }
                else
                {
                    // overwrite Leaf. This is executed once. Next step is after(...).
                    // Note: here might be two transient ids in the pair
                    m_checkout->mutable_transientoidstoorigin()
                            ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(transOid, "")); //TODO: history of createLeaf lost, if ther was one
                    upnsPair<StatusCode, ObjectId> soid = storeObject(createLeaf, transOid);
                    if(!upnsIsOk(soid.first)) return false;
                    leafOid = soid.second;
                    leafWasStored = true;
                }
            }
            else
            {
                // copy
                if(!isLast || (isLast && createLeaf == NULL))
                {
                    // copy existing tree under transient oid
                    upnsSharedPointer<Tree> tree(m_serializer->getTree(oid));
                    if(tree == NULL)
                    {
                        log_error("Segment of path was not a tree");
                        return false;
                    }
                    m_checkout->mutable_transientoidstoorigin()
                            ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(transOid, oid));
                    exclusiveTreePath.push_back( tree );
                    current = tree;
                }
                else
                {
                    // overwrite Leaf with copy. This is executed once. Next step is after(...).
                    m_checkout->mutable_transientoidstoorigin()
                            ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(transOid, oid));
                    //createLeaf->set_id(nextOid);
                    upnsPair<StatusCode, ObjectId> soid = storeObject(createLeaf, transOid);
                    if(!upnsIsOk(soid.first)) return false;
                    leafOid = soid.second;
                    leafWasStored = true;
                }
            }

        }
        return true; // continue thru path
    },
    [&](upnsString seg, size_t idx, bool isLast)
    {
        ObjectId transOid;
        if(leafWasStored && isLast) // leaf already created in "before"
        {
            transOid = leafOid;
            assert(!transOid.empty());
        }
        else
        {
            current = exclusiveTreePath.back();
            exclusiveTreePath.pop_back();
            transOid = m_name + "/" + p.substr(0, idx);
            upnsPair<StatusCode, ObjectId> soid = m_serializer->storeTreeTransient(current, transOid);
            transOid = soid.second;
            if(!upnsIsOk(soid.first)) return false; // must never happen. leads to inconsistent data. TODO: rollback
            if(exclusiveTreePath.empty())
            {
                assert(idx == 0);
                return false;
            }
        }
        upnsSharedPointer<Tree> parent = exclusiveTreePath.back();
        ObjectReference oref;
        oref.set_id(transOid);
        //oref.set_lastchange(QDateTime::currentDateTime().toMSecsSinceEpoch()); //< breaks hashing
        assert(parent);
        parent->mutable_refs()
                ->insert( ::google::protobuf::MapPair< ::std::string, ::upns::ObjectReference>( seg, oref));
        return true;
    });

    // store/create root
    upnsSharedPointer<Tree> obj = exclusiveTreePath.back();
    exclusiveTreePath.pop_back();
    upnsPair<StatusCode, ObjectId> soid = m_serializer->storeTreeTransient(obj, m_name + "/");
    if(!upnsIsOk(soid.first)) return soid.first;

    // update checkout commit
    StatusCode s = m_serializer->storeCheckoutCommit(m_checkout, m_name);
    return s;
}

}
#endif
