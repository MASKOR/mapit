#ifndef CHECKOUTIMPL_H
#define CHECKOUTIMPL_H

#include "upns_typedefs.h"
#include "upns_logging.h"
#include "services.pb.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include "serialization/abstractmapserializer.h"
#include "entitydata.h"
#include "versioning/checkout.h"
#include "modules/versioning/checkoutraw.h"
#include "util.h"
#include <functional>

namespace upns
{

// PathInternal contains the checkout name at the beginning
// The corresponing types should be used. However there is no "typechecking" here.
typedef Path PathInternal;
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
    CheckoutImpl(upns::upnsSharedPointer<AbstractMapSerializer> serializer, upnsSharedPointer<CheckoutObj> checkoutCommit, upnsString name, const upnsString branchname = "");

    virtual bool isInConflictMode();
    virtual upnsVec< upnsSharedPointer<Conflict> > getPendingConflicts();
    virtual upnsSharedPointer<Tree> getRoot();
    virtual upnsSharedPointer<Tree> getTree(const Path &path);
    virtual upnsSharedPointer<Entity> getEntity(const Path &path);
    virtual MessageType typeOfObject(const Path &path);
    virtual upnsSharedPointer<Tree> getTreeConflict(const ObjectId &objectId);
    virtual upnsSharedPointer<Entity> getEntityConflict(const ObjectId &objectId);
    virtual OperationResult doOperation(const OperationDescription &desc);
    virtual OperationResult doUntraceableOperation(const OperationDescription &desc, std::function<upns::StatusCode(OperationEnvironment *)> operate);

    virtual upnsSharedPointer<AbstractEntitydata> getEntitydataReadOnly(const Path &path);
    virtual upnsSharedPointer<AbstractEntitydata> getEntitydataReadOnlyConflict(const ObjectId &entityId);
    virtual upnsSharedPointer<AbstractEntitydata> getEntitydataForReadWrite(const Path &path);

    virtual StatusCode storeTree(const Path &path, upnsSharedPointer<Tree> tree);
    virtual StatusCode storeEntity(const Path &path, upnsSharedPointer<Entity> entity);

    virtual void setConflictSolved(const Path &path, const ObjectId &oid);

    virtual upnsSharedPointer<Branch> getParentBranch();
    virtual upnsVec<CommitId> getParentCommitIds();

    template <typename T>
    inline upnsString generateTransientOid(const upnsSharedPointer<T> &obj);

    StatusCode depthFirstSearch(std::function<bool(upnsSharedPointer<Commit>, const ObjectReference&, const Path&)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectReference&, const Path&)> afterCommit,
                                std::function<bool(upnsSharedPointer<Tree>, const ObjectReference&, const Path&)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectReference&, const Path&)> afterTree,
                                std::function<bool(upnsSharedPointer<Entity>, const ObjectReference&, const Path&)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectReference&, const Path&)> afterEntity);
    upnsSharedPointer<CheckoutObj> getCheckoutObj();
    const upnsString& getName() const;
private:

    upnsSharedPointer<Tree> getTree(const ObjectReference &ref);
    upnsSharedPointer<Entity> getEntity(const ObjectReference &ref);
    MessageType typeOfObject(const ObjectReference &ref);
    /**
     * @brief objectReferenceOfChild Used to get reference of children
     * @param tree parent
     * @param name of the child (path segment, without slashes)
     * @return Oid or empty oid
     */
    ObjectReference objectReferenceOfChild(upnsSharedPointer<Tree> tree, const std::string &name);

    /**
     * @brief oidForPath Used to convert path to oids.
     * @param path beginning with root dir of checkout. Checkout must not be part of path. Can have leading or trailing slashes.
     * @return Oid or empty oid
     */
    ObjectReference objectReferenceForPath(const Path &path);

    // helper, used to ensure slashes/no slashes at beginning and end
    Path preparePath(const Path &path);
    Path preparePathFilename(const Path &path);

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
    upnsPair<StatusCode, PathInternal> storeObject(upnsSharedPointer<T> leafObject, const PathInternal &pathInternal);

    bool forEachPathSegment(const Path &path,
                                  std::function<bool(upnsString, size_t, bool)> before, std::function<bool(upnsString, size_t, bool)> after, const int start = 0);

//    /**
//     * @brief Depth first search for Commit, Tree and Entity.
//     * Does not work for branches. Does not visit Entitydata (must be done manually).
//     * If "before" returns false, "after" will not be executed.
//     */
//    StatusCode depthFirstSearch(upnsSharedPointer<Commit> obj, const ObjectId& oid, const Path &path,
//                                std::function<bool(upnsSharedPointer<Commit>, const ObjectReference &)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectReference &)> afterCommit,
//                                std::function<bool(upnsSharedPointer<Tree>, const ObjectReference &)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectReference &)> afterTree,
//                                std::function<bool(upnsSharedPointer<Entity>, const ObjectReference &)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectReference &)> afterEntity);

//    StatusCode depthFirstSearch(upnsSharedPointer<Tree> obj, const ObjectId& oid, const Path &path,
//                                std::function<bool(upnsSharedPointer<Commit>, const ObjectReference &)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectReference &)> afterCommit,
//                                std::function<bool(upnsSharedPointer<Tree>, const ObjectReference &)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectReference &)> afterTree,
//                                std::function<bool(upnsSharedPointer<Entity>, const ObjectReference &)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectReference &)> afterEntity);

//    StatusCode depthFirstSearch(upnsSharedPointer<Entity> obj, const ObjectId& oid, const Path &path,
//                                std::function<bool(upnsSharedPointer<Commit>, const ObjectReference &)> beforeCommit, std::function<bool(upnsSharedPointer<Commit>, const ObjectReference &)> afterCommit,
//                                std::function<bool(upnsSharedPointer<Tree>, const ObjectReference &)> beforeTree, std::function<bool(upnsSharedPointer<Tree>, const ObjectReference &)> afterTree,
//                                std::function<bool(upnsSharedPointer<Entity>, const ObjectReference &)> beforeEntity, std::function<bool(upnsSharedPointer<Entity>, const ObjectReference &)> afterEntity);

    upns::upnsSharedPointer<AbstractMapSerializer> m_serializer;

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
    bool rootMissing = !m_checkout->rollingcommit().has_root() || m_checkout->rollingcommit().root().id().empty() && m_checkout->rollingcommit().root().path().empty();
    // root is not transient and must be copied to edit.
    bool rootNotExclusive = (!rootMissing) && (!m_checkout->rollingcommit().root().id().empty() && m_checkout->rollingcommit().root().path().empty());
    //bool rootNotExclusive = m_checkout->transientoidstoorigin().count(m_checkout->rollingcommit().root()) == 0;
    assert(rootMissing || (rootNotExclusive == (m_checkout->transientoidstoorigin().count(m_checkout->rollingcommit().root().path()) == 0)));
    // if there is no root directory, checkout must be empty and have nothing transient
    assert(!rootMissing || rootMissing && m_checkout->transientoidstoorigin_size() == 0);
    if(rootMissing || rootNotExclusive)
    {
        // create/copy root for checkout
        ::std::string originOid;
        if(rootMissing)
        {
            // create new root
            originOid = "";
            current = upnsSharedPointer<Tree>(new Tree);
        }
        else
        {
            // copy to make exclusive
            originOid = m_checkout->rollingcommit().root().id();
            current = m_serializer->getTree(originOid);
        }
        m_checkout->mutable_rollingcommit()->mutable_root()->set_path(m_name + "/");
        m_checkout->mutable_transientoidstoorigin()
                ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(m_checkout->rollingcommit().root().path(), originOid));
        exclusiveTreePath.push_back( current );
    }
    else
    {
        // root exists and can exclusively be altered
        PathInternal path = m_checkout->rollingcommit().root().path();
        current = m_serializer->getTreeTransient(path);
        exclusiveTreePath.push_back( current );
    }

    bool leafWasStored = false;
    PathInternal leafPathIntenal;
    forEachPathSegment(p,
    [&](upnsString seg, size_t idx, bool isLast)
    {
        if(current == NULL) return false; // can not go futher
        if(seg.empty()) return false; // "//" not allowed
        ObjectReference ref = objectReferenceOfChild(current, seg);
        PathInternal pathInternal = m_name + "/" + p.substr(0,idx);
        // 6 cases:
        // 1+2) no oid yet          -> create tree/entity, put in vector to update and store. (for leaf: create directly)
        // 3+4) exclusive oids      -> put in vector to update and store (for leaf: create directly)
        // 5+6) non-exclusive oids  -> put copy in vector to update and store (for leaf: create directly)
        assert(ref.id().empty() || ref.path().empty()); // both must not be set at the same time.
        bool segNotExistant = ref.id().empty() && ref.path().empty();
        if(segNotExistant)
        {
            // create new
            if(!isLast || (isLast && createLeaf == NULL))
            {
                // append tree
                upnsSharedPointer<Tree> tree(new Tree);
                exclusiveTreePath.push_back( tree );
                m_checkout->mutable_transientoidstoorigin()
                        ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(pathInternal, ""));
                current = tree;
            }
            else
            {
                // create Leaf. This is executed once. Next step is after(...).
                upnsPair<StatusCode, PathInternal> pathStatus = storeObject(createLeaf, pathInternal);
                leafPathIntenal = pathStatus.second;
                if(!upnsIsOk(pathStatus.first)) return false;
                m_checkout->mutable_transientoidstoorigin()
                        ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(pathStatus.second, "")); //TODO: history of createLeaf lost, if ther was one

                leafWasStored = true;
            }
        }
        else
        {
            // use or copy existing
            bool segExclusive = ref.id().empty() && !ref.path().empty();
            //bool segExclusive = m_checkout->transientoidstoorigin().count(oid) != 0;
            assert(segExclusive == (m_checkout->transientoidstoorigin().count(ref.path()) != 0));
            if(segExclusive)
            {
                // use existing
                if(!isLast || (isLast && createLeaf == NULL))
                {
                    // put tree in vector and do nothing
                    upnsSharedPointer<Tree> tree(m_serializer->getTreeTransient(ref.path()));
                    if(tree == NULL)
                    {
                        log_error("Segment of path was not a tree or path was wrong");
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
                            ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(pathInternal, "")); //TODO: history of createLeaf lost, if ther was one
                    upnsPair<StatusCode, PathInternal> statusPath = storeObject(createLeaf, pathInternal);
                    if(!upnsIsOk(statusPath.first)) return false;
                    leafPathIntenal = statusPath.second;
                    leafWasStored = true;
                }
            }
            else
            {
                // copy
                assert(!ref.id().empty() && ref.path().empty());
                if(!isLast || (isLast && createLeaf == NULL))
                {
                    // copy existing tree under transient oid
                    upnsSharedPointer<Tree> tree(m_serializer->getTree(ref.id()));
                    if(tree == NULL)
                    {
                        log_error("Segment of path was not a tree or corrupt oids");
                        return false;
                    }
                    m_checkout->mutable_transientoidstoorigin()
                            ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(pathInternal, ref.id()));
                    exclusiveTreePath.push_back( tree );
                    current = tree;
                }
                else
                {
                    // overwrite Leaf with copy. This is executed once. Next step is after(...).
                    m_checkout->mutable_transientoidstoorigin()
                            ->insert(::google::protobuf::MapPair< ::std::string, ::std::string>(pathInternal, ref.id()));
                    //createLeaf->set_id(nextOid);
                    upnsPair<StatusCode, PathInternal> status_path = storeObject(createLeaf, pathInternal);
                    if(!upnsIsOk(status_path.first)) return false;
                    leafPathIntenal = status_path.second;
                    leafWasStored = true;
                }
            }

        }
        return true; // continue thru path
    },
    [&](upnsString seg, size_t idx, bool isLast)
    {
        PathInternal pathInternal;
        if(leafWasStored && isLast) // leaf already created in "before"
        {
            assert(!leafPathIntenal.empty());
            pathInternal = leafPathIntenal;
        }
        else
        {
            current = exclusiveTreePath.back();
            exclusiveTreePath.pop_back();
            pathInternal = m_name + "/" + p.substr(0, idx);
            upnsPair<StatusCode, PathInternal> status_path = m_serializer->storeTreeTransient(current, pathInternal);
            pathInternal = status_path.second;
            if(!upnsIsOk(status_path.first)) { assert(false); return false;} // must never happen. leads to inconsistent data. TODO: rollback
            if(exclusiveTreePath.empty())
            {
                assert(idx == 0);
                return false;
            }
        }
        upnsSharedPointer<Tree> parent = exclusiveTreePath.back();
        ObjectReference oref;
        oref.set_path(pathInternal);
        //oref.set_lastchange(QDateTime::currentDateTime().toMSecsSinceEpoch()); //< breaks hashing
        assert(parent);
        parent->mutable_refs()
                ->insert( ::google::protobuf::MapPair< ::std::string, ::upns::ObjectReference>( seg, oref));
        return true;
    });

    // store/create root
    upnsSharedPointer<Tree> obj = exclusiveTreePath.back();
    exclusiveTreePath.pop_back();
    upnsPair<StatusCode, PathInternal> status_path = m_serializer->storeTreeTransient(obj, m_name + "/");
    if(!upnsIsOk(status_path.first)) return status_path.first;

    // update checkout commit
    StatusCode s = m_serializer->storeCheckoutCommit(m_checkout, m_name);
    return s;
}

}
#endif
