#ifndef QMLCHECKOUT
#define QMLCHECKOUT

#include <QtCore>
#include "upns.h"
#include "qmltree.h"
#include "qmlentity.h"
#include "qmlcommit.h"
#include "qmlcheckout.h"
#include "qmlbranch.h"
#include "qmlentitydata.h"
#include "versioning/checkout.h"
#include "libs/upns_interface/services.pb.h"
#include <QJsonObject>

class QmlCheckout : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool isInConflictMode READ isInConflictMode NOTIFY isInConflictModeChanged)

public:
    QString doOperation(QString operatorname, const QJsonObject &desc);

    //upnsVec< upnsSharedPointer<Conflict> > getPendingConflicts() = 0;

    void setConflictSolved(QString path, QString oid);

    QmlTree* getRoot();

    QmlTree* getTreeConflict(QString objectId);

    QmlEntity* getEntityConflict(QString objectId);

    QmlTree* getTree(QString path);

    QmlEntity* getEntity(QString path);

    QmlBranch* getParentBranch();

    QString getParentCommitIds();

    QmlEntitydata* getEntityDataReadOnly(QString entityId);

    QmlEntitydata* getEntityDataReadOnlyConflict(QString entityId);

    bool isInConflictMode() const;

Q_SIGNALS:
    void isInConflictModeChanged(bool isInConflictMode);

protected:
    upns::upnsSharedPointer<upns::Checkout> m_checkout;
private:

    bool m_isInConflictMode;
};

#endif
