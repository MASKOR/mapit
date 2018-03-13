/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2016-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ABSTRACTSERIALIZER_H
#define ABSTRACTSERIALIZER_H

#include <mapit/typedefs.h>
#include <mapit/msgs/services.pb.h>
#include <mapit/operators/serialization/abstractentitydataprovider.h>
#include <mapit/entitydata.h>
#include <mapit/errorcodes.h>

namespace mapit
{
// Path with leading checkout name
typedef Path PathInternal;
/**
 * @brief The AbstractSerializer class capsulates data access. From outside maps and entities can be read/written.
 * However, the enduser should use a more convenient interface (versioning). This class is abstract and must be implemented by a
 * concrete class which has access to a storage (harddrive, files, network, database, other MapSerializer, ...).
 * Mapmanager is the only code that should know about this interface. It is located in the "include" folder for tests and stubs.
 */
class AbstractSerializer
{
public:
    virtual ~AbstractSerializer() {}
    virtual bool canRead() = 0;
    virtual bool canWrite() = 0;

    virtual std::shared_ptr<mapit::msgs::Tree> getTree(const ObjectId &oid) = 0;
    virtual std::shared_ptr<mapit::msgs::Tree> getTreeTransient(const PathInternal &transientId) = 0;
    // Note: storing and creating is only distinguished for transient oid (paths). When
    //       Hashes are used, the system does not know if a tree/entity with the same hash
    //       already exists or if it is a new tree/entity
    virtual std::pair<StatusCode, ObjectId> storeTree(std::shared_ptr<mapit::msgs::Tree> &obj) = 0;
    virtual std::pair<StatusCode, ObjectId> storeTreeTransient(std::shared_ptr<mapit::msgs::Tree> &obj, const PathInternal &transientId) = 0;
    //virtual StatusCode createTree(std::shared_ptr<Tree> &obj) = 0;
    virtual StatusCode removeTreeTransient(const ObjectId &oid) = 0;

    virtual std::shared_ptr<mapit::msgs::Entity> getEntity(const ObjectId oid) = 0;
    virtual std::shared_ptr<mapit::msgs::Entity> getEntityTransient(const PathInternal path) = 0;
    virtual std::pair<StatusCode, ObjectId> storeEntity(std::shared_ptr<mapit::msgs::Entity> &obj) = 0;
    virtual std::pair<StatusCode, ObjectId> storeEntityTransient(std::shared_ptr<mapit::msgs::Entity> &obj, const PathInternal &transientId) = 0;
    //virtual StatusCode createEntity(std::shared_ptr<Entity> &obj) = 0;
    virtual StatusCode removeEntityTransient(const PathInternal &oid) = 0;

    virtual std::shared_ptr<mapit::msgs::Commit> getCommit(const ObjectId &oid) = 0;
    //virtual StatusCode storeCommit(std::shared_ptr<Commit> &obj) = 0;
    virtual std::pair<StatusCode, ObjectId> createCommit(std::shared_ptr<mapit::msgs::Commit> &obj) = 0;
    virtual StatusCode removeCommit(const ObjectId &oid) = 0;

    virtual std::vector< std::string > listCheckoutNames() = 0;
    virtual std::vector< std::shared_ptr<mapit::msgs::CheckoutObj> > listCheckouts() = 0;
    virtual std::shared_ptr<mapit::msgs::CheckoutObj> getCheckoutCommit(const std::string &name) = 0;
    virtual StatusCode storeCheckoutCommit(std::shared_ptr<mapit::msgs::CheckoutObj> &obj, const std::string &name) = 0;
    virtual StatusCode createCheckoutCommit(std::shared_ptr<mapit::msgs::CheckoutObj> &obj, const std::string &name) = 0;
    virtual StatusCode removeCheckout(const std::string &name) = 0;

    virtual std::vector< std::shared_ptr<mapit::msgs::Branch> > listBranches() = 0;
    virtual std::shared_ptr<mapit::msgs::Branch> getBranch(const std::string &name) = 0;
    virtual StatusCode storeBranch(std::shared_ptr<mapit::msgs::Branch> &obj, const std::string &name) = 0;
    virtual StatusCode createBranch(std::shared_ptr<mapit::msgs::Branch> &obj, const std::string &name) = 0;
    virtual StatusCode removeBranch(const std::string &name) = 0;

    virtual bool existsStreamProvider(const ObjectId &entityId) = 0;
    virtual bool existsStreamProviderTransient(const Path &path) = 0;
    virtual std::shared_ptr<AbstractEntitydataProvider> getStreamProvider(const ObjectId &entityId, bool canRead = true) = 0;
    virtual std::shared_ptr<AbstractEntitydataProvider> getStreamProviderTransient(const Path &path, bool canRead = true, bool canWrite = false) = 0;


    virtual mapit::msgs::MessageType typeOfObject(const ObjectId &oid) = 0;
    virtual mapit::msgs::MessageType typeOfObjectTransient(const PathInternal &path) = 0;
    virtual bool exists(const ObjectId &oidOrName) = 0;

    virtual std::pair<StatusCode, ObjectId> persistTransientEntitydata(const PathInternal &path) = 0;
//    virtual bool isTree(const ObjectId &oid) = 0;
//    virtual bool isEntity(const ObjectId &oid) = 0;
//    virtual bool isCommit(const CommitId &oid) = 0;
//    virtual bool isCheckout(const CommitId &oid) = 0;
//    virtual bool isBranch(const CommitId &oid) = 0;
    /**
     * @brief cleanUp Collects Grabage. Orphan Objects, not reachable by any branch are removed.
     * @return
     */
    virtual StatusCode cleanUp() = 0;
#ifdef MAPIT_DEBUG
    virtual void debugDump() = 0;
#endif
};

}
#endif
