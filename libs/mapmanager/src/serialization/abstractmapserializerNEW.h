#ifndef ABSTRACTMAPSERIALIZER_H
#define ABSTRACTMAPSERIALIZER_H

#include "upns_globals.h"
#include "services.pb.h"
#include "abstractentitydatastreamprovider.h"
#include "entitydata.h"
#include "error.h"

namespace upns
{

/**
 * @brief The AbstractMapSerializer class capsulates data access. From outside maps and entities can be read/written.
 * However, the enduser use a more convenient interface (versioning). This class is abstract and must be implemented by a
 * concrete class which has access to a storage (harddrive, files, netrwork, database, other MapSerializer, ...).
 */
class AbstractMapSerializer
{
public:
    AbstractMapSerializer();
    virtual ~AbstractMapSerializer();

    bool canRead() = 0;
    bool canWrite() = 0;

    virtual upnsSharedPointer<Treeish> getTreeish(const ObjectId &oid) = 0;
    virtual StatusCode storeTreeish(upnsSharedPointer<Treeish> &treeish) = 0;
    virtual StatusCode createTreeish(upnsSharedPointer<Treeish> &treeish) = 0;
    virtual StatusCode removeTreeish(const ObjectId &oid) = 0;

    virtual upnsSharedPointer<Entity> getEntity(const ObjectId oid) = 0;
    virtual StatusCode storeEntity(upnsSharedPointer<Entity> &entity) = 0;
    virtual StatusCode createEntity(upnsSharedPointer<Entity> &entity) = 0;
    virtual StatusCode removeEntity(const ObjectId &oid) = 0;

    virtual upnsSharedPointer<Commit> getCommit(const ObjectId &oid) = 0;
    virtual StatusCode storeCommit(upnsSharedPointer<Commit> &obj) = 0;
    virtual StatusCode createCommit(upnsSharedPointer<Commit> &obj) = 0;
    virtual StatusCode removeCommit(const ObjectId &oid) = 0;

    virtual upnsVec< upnsSharedPointer<Commit> > listCheckouts() = 0;
    virtual upnsSharedPointer<Commit> getCheckoutCommit(const ObjectId &oid) = 0;
    virtual StatusCode storeCheckoutCommit(upnsSharedPointer<Commit> &obj) = 0;
    virtual StatusCode createCheckoutCommit(upnsSharedPointer<Commit> &obj) = 0;
    virtual StatusCode removeCheckoutCommit(const ObjectId &oid) = 0;

    virtual upnsVec< upnsSharedPointer<Branch> > listBranches() = 0;
    virtual upnsSharedPointer<Branch> getBranch(const ObjectId &oid) = 0;
    virtual StatusCode storeBranch(upnsSharedPointer<Branch> &obj) = 0;
    virtual StatusCode createBranch(upnsSharedPointer<Branch> &obj) = 0;
    virtual StatusCode removeBranch(const ObjectId &oid) = 0;

    virtual upnsSharedPointer<AbstractEntityDataStreamProvider> getStreamProvider(const ObjectId &entityId) = 0;

    /**
     * @brief cleanUp Collects Grabage. Orphan Objects, not reachable by any branch are removed.
     * @return
     */
    virtual StatusCode cleanUp() = 0;

private:
    upnsSharedPointer<AbstractEntityData> wrapEntityOfType(LayerType type,
                                                         upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider);
    upnsSharedPointer<AbstractEntityData> wrapEntityOfType(upnsString layertypeName,
                                                         upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider);
};

}
#endif
