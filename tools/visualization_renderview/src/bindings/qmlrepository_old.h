//#ifndef QMLREPOSITORY_H
//#define QMLREPOSITORY_H

//#include <QtCore>
//#include <QJsonObject>
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

//    // upnsSharedPointer<AbstractEntityData> getEntityDataReadOnly(const ObjectId &oid);
//    QmlCheckout *checkout(QString commitIdOrBranchname, QString name);
//    QmlCheckout *checkout(QString checkoutName);
//    // StatusCode deleteCheckoutForced(const upnsString &checkoutName);
//    // CommitId commit(const upnsSharedPointer<Checkout> checkout, const upnsString msg);
//    // upnsVec< upnsSharedPointer<Branch> > getBranches();
//    // StatusCode push(Repository &repo);
//    // StatusCode pull(Repository &repo);
//    // CommitId parseCommitRef(const upnsString &commitRef);
//    // upnsSharedPointer<Checkout> merge(const CommitId mine, const CommitId theirs, const CommitId base);
//    // upnsVec< upnsPair<CommitId, ObjectId> > ancestors(const CommitId &commitId, const ObjectId &objectId, const int level = 0);
//    Q_INVOKABLE bool canRead();
//    Q_INVOKABLE bool canWrite();
//};

//#endif
