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
