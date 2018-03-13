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

//#ifndef QMLREPOSITORY_H
//#define QMLREPOSITORY_H

//#include <QtCore>
//#include <QtCore/QJsonObject>
//#include "libs/mapmanager/include/versioning/repository.h"
//#include "qmltree.h"
//#include "qmlentity.h"
//#include "qmlcommit.h"
//#include "qmlcheckoutinfo.h"
//#include "qmlcheckout.h"
//#include "qmlbranch.h"

//class QmlRepository : public QObject
//{
//    Q_OBJECT

//public:

//    Q_INVOKABLE QStringList listCheckoutNames();

//    Q_INVOKABLE QmlTree *getTree(QString oid);
//    Q_INVOKABLE QmlEntity *getEntity(QString oid);
//    Q_INVOKABLE QmlCommit *getCommit(QString oid);
//    Q_INVOKABLE QmlCheckoutInfo *getCheckout(QString name);
//    Q_INVOKABLE QmlBranch *getBranch(QString name);
//    //MessageType typeOfObject(const ObjectId &oid);

//    // std::shared_ptr<AbstractEntitydata> getEntitydataReadOnly(const ObjectId &oid);
//    QmlCheckout *checkout(QString commitIdOrBranchname, QString name);
//    QmlCheckout *checkout(QString checkoutName);
//    // StatusCode deleteCheckoutForced(const std::string &checkoutName);
//    // CommitId commit(const std::shared_ptr<Checkout> checkout, const std::string msg);
//    // std::vector< std::shared_ptr<Branch> > getBranches();
//    // StatusCode push(Repository &repo);
//    // StatusCode pull(Repository &repo);
//    // CommitId parseCommitRef(const std::string &commitRef);
//    // std::shared_ptr<Checkout> merge(const CommitId mine, const CommitId theirs, const CommitId base);
//    // std::vector< std::pair<CommitId, ObjectId> > ancestors(const CommitId &commitId, const ObjectId &objectId, const int level = 0);
//    Q_INVOKABLE bool canRead();
//    Q_INVOKABLE bool canWrite();
//};

//#endif
