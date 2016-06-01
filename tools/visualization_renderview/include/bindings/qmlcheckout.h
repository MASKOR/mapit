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
    QmlCheckout();
    QmlCheckout( upns::upnsSharedPointer<upns::Checkout> &co );

    Q_INVOKABLE QString doOperation(QString operatorname, const QJsonObject &desc);

    //upnsVec< upnsSharedPointer<Conflict> > getPendingConflicts() = 0;

    Q_INVOKABLE void setConflictSolved(QString path, QString oid);

    Q_INVOKABLE QmlTree* getRoot();

    Q_INVOKABLE QmlTree* getTreeConflict(QString objectId);

    Q_INVOKABLE QmlEntity* getEntityConflict(QString objectId);

    Q_INVOKABLE QmlTree* getTree(QString path);

    Q_INVOKABLE QmlEntity* getEntity(QString path);

    Q_INVOKABLE QmlBranch* getParentBranch();

    Q_INVOKABLE QStringList getParentCommitIds();

    Q_INVOKABLE QmlEntitydata* getEntitydataReadOnly(QString path);

    Q_INVOKABLE QmlEntitydata* getEntitydataReadOnlyConflict(QString entityId);

    Q_INVOKABLE bool isInConflictMode() const;

    upns::upnsSharedPointer<upns::Checkout> getCheckoutObj() { return m_checkout; }
Q_SIGNALS:
    void isInConflictModeChanged(bool isInConflictMode);

protected:
    upns::upnsSharedPointer<upns::Checkout> m_checkout;
private:

    bool m_isInConflictMode;
};

#endif
