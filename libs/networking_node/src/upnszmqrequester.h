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

#ifndef UPNSZMQREQUESTER_H
#define UPNSZMQREQUESTER_H

#include <string>
#include <upns/versioning/repository.h>

namespace upns {

class ZmqRequesterPrivate;

///
/// \brief The upns::ZmqRequester class
/// Implements the basic Repository Interface and will send requests over network
///

class ZmqRequester : public upns::Repository
{
public:
    ZmqRequester(Repository* cache, std::string urlOutgoingRequests = std::string(), bool operationsLocal = false);
    ~ZmqRequester();

    // Repository interface
public:
    std::vector<std::string> listCheckoutNames();
    std::shared_ptr<Tree> getTree(const ObjectId &oid);
    std::shared_ptr<Entity> getEntity(const ObjectId &oid);
    std::shared_ptr<Commit> getCommit(const ObjectId &oid);
    std::shared_ptr<CheckoutObj> getCheckoutObj(const std::string &name);
    std::shared_ptr<Branch> getBranch(const std::string &name);
    MessageType typeOfObject(const ObjectId &oid);
    std::shared_ptr<AbstractEntitydata> getEntitydataReadOnly(const ObjectId &oid);
    std::shared_ptr<Checkout> createCheckout(const CommitId &commitIdOrBranchname, const std::string &name);
    std::shared_ptr<Checkout> getCheckout(const std::string &checkoutName);
    StatusCode deleteCheckoutForced(const std::string &checkoutName);
    CommitId commit(const std::shared_ptr<Checkout> checkout, std::string msg);
    std::vector<std::shared_ptr<Branch> > getBranches();
    StatusCode push(Repository &repo);
    StatusCode pull(Repository &repo);
    CommitId parseCommitRef(const std::string &commitRef);
    std::shared_ptr<Checkout> merge(const CommitId mine, const CommitId theirs, const CommitId base);
    std::vector<std::pair<CommitId, ObjectId> > ancestors(const CommitId &commitId, const ObjectId &objectId, const int level);
    bool canRead();
    bool canWrite();

private:
    ZmqRequesterPrivate *m_d;
};

}

#endif // UPNSZMQREQUESTER_H
