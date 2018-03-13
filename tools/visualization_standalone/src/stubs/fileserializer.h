/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

//#ifndef FILESERIALIZER_H
//#define FILESERIALIZER_H

//#include "serialization/abstractmapserializer.h"
//namespace upns
//{

//class FileSerializer : public upns::AbstractSerializer
//{


//    // AbstractSerializer interface
//public:
//    bool canRead();
//    bool canWrite();
//    std::shared_ptr<Tree> getTree(const ObjectId &oid);
//    std::pair<StatusCode, ObjectId> storeTree(std::shared_ptr<Tree> &obj);
//    StatusCode createTree(std::shared_ptr<Tree> &obj);
//    StatusCode removeTree(const ObjectId &oid);
//    std::shared_ptr<Entity> getEntity(const ObjectId oid);
//    std::pair<StatusCode, ObjectId> storeEntity(std::shared_ptr<Entity> &obj);
//    StatusCode createEntity(std::shared_ptr<Entity> &obj);
//    StatusCode removeEntity(const ObjectId &oid);
//    std::shared_ptr<Commit> getCommit(const ObjectId &oid);
//    StatusCode storeCommit(std::shared_ptr<Commit> &obj);
//    std::pair<StatusCode, ObjectId> createCommit(std::shared_ptr<Commit> &obj);
//    StatusCode removeCommit(const ObjectId &oid);
//    std::vector<std::string> listCheckoutNames();
//    std::vector<std::shared_ptr<CheckoutObj> > listCheckouts();
//    std::shared_ptr<CheckoutObj> getCheckoutCommit(const std::string &name);
//    StatusCode storeCheckoutCommit(std::shared_ptr<CheckoutObj> &obj, const std::string &name);
//    StatusCode createCheckoutCommit(std::shared_ptr<CheckoutObj> &obj, const std::string &name);
//    StatusCode removeCheckoutCommit(const std::string &name);
//    std::vector<std::shared_ptr<Branch> > listBranches();
//    std::shared_ptr<Branch> getBranch(const std::string &name);
//    StatusCode storeBranch(std::shared_ptr<Branch> &obj, const std::string &name);
//    StatusCode createBranch(std::shared_ptr<Branch> &obj, const std::string &name);
//    StatusCode removeBranch(const std::string &name);
//    std::shared_ptr<AbstractEntitydataProvider> getStreamProvider(const ObjectId &entityId, bool canRead, bool canWrite);
//    MessageType typeOfObject(const ObjectId &oidOrName);
//    bool exists(const ObjectId &oidOrName);
//    StatusCode cleanUp();
//};

//}
//#endif
