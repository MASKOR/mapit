/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef REPOSITORYIMPL_H
#define REPOSITORYIMPL_H

#include <mapit/typedefs.h>
#include <mapit/msgs/services.pb.h>
#include <mapit/operators/serialization/abstractentitydataprovider.h>
#include <mapit/entitydata.h>
#include <mapit/versioning/checkout.h>
#include <mapit/versioning/repository.h>

namespace mapit
{
class RepositoryPrivate;
class AbstractSerializer;
class RepositoryImpl : public Repository
{
public:
    RepositoryImpl(std::shared_ptr<mapit::AbstractSerializer> serializer);
    virtual ~RepositoryImpl();

    std::vector<std::string> listCheckoutNames();

    std::shared_ptr<Tree>         getTree(const ObjectId &oid);
    std::shared_ptr<Entity>       getEntity(const ObjectId &oid);
    std::shared_ptr<Commit>       getCommit(const ObjectId &oid);
    std::shared_ptr<CheckoutObj>  getCheckoutObj(const std::string &name);
    std::shared_ptr<Branch>       getBranch(const std::string &name);

    MessageType typeOfObject(const ObjectId &oid);

    std::shared_ptr<AbstractEntitydata> getEntitydataReadOnly(const ObjectId &oid);

    std::shared_ptr<Checkout> createCheckout(const CommitId &commitIdOrBranchname, const std::string &name);
    std::shared_ptr<Checkout> getCheckout(const std::string &checkoutName);
    StatusCode                  deleteCheckoutForced(const std::string &checkoutName);

    CommitId commit(std::shared_ptr<Checkout> &checkout, std::string msg);

    std::vector< std::shared_ptr<Branch> > getBranches();

    StatusCode push(Repository &repo);
    StatusCode pull(Repository &repo);

    CommitId parseCommitRef(const std::string &commitRef);

    std::shared_ptr<Checkout> merge(const CommitId mine, const CommitId theirs, const CommitId base);

    std::vector< std::pair<CommitId, ObjectId> > ancestors(const CommitId &commitId, const ObjectId &objectId, const int level = 0);

    bool canRead();
    bool canWrite();
private:
    RepositoryPrivate* m_p;
};

}
#endif
