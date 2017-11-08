#ifndef QMLCHECKOUT
#define QMLCHECKOUT

#include <QtCore>

#include "qmltree.h"
#include "qmlentity.h"
#include "qmlcommit.h"
#include "qmlcheckout.h"
#include "qmlbranch.h"
#include "qmlentitydata.h"
#include "qmlrepository.h"
#include <upns/versioning/checkout.h>
#include <mapit/msgs/services.pb.h>
#include <QtCore/QJsonObject>

class QmlRepository;

class QmlCheckout : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool isInConflictMode READ isInConflictMode NOTIFY isInConflictModeChanged)
    Q_PROPERTY(QmlRepository* repository READ repository WRITE setRepository NOTIFY repositoryChanged)
    Q_PROPERTY(QString name READ name WRITE setName NOTIFY nameChanged)
    Q_PROPERTY(QStringList entities READ entities NOTIFY entitiesChanged)
public:
    QmlCheckout();
    QmlCheckout( std::shared_ptr<upns::Checkout> &co, QmlRepository* repo = NULL, QString name = "" );

    Q_INVOKABLE QString doOperation(QString operatorname, const QJsonObject &desc);

    //std::vector< std::shared_ptr<Conflict> > getPendingConflicts() = 0;

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

    std::shared_ptr<upns::Checkout> getCheckoutObj() { return m_checkout; }

    QmlRepository* repository() const;

    QString name() const;

    QStringList entities() const;

    //TODO: Put this to a another class. This introduces dependecy
    //      to tfs from checkout (core->entitytype).

    Q_INVOKABLE QStringList getFrameIds();
public Q_SLOTS:
    void setRepository(QmlRepository* repository);
    void setName(QString name);

Q_SIGNALS:
    void isInConflictModeChanged(bool isInConflictMode);

    void repositoryChanged(QmlRepository* repository);

    void nameChanged(QString name);

    void internalCheckoutChanged(QmlCheckout *co);

    void entitiesChanged(QStringList entities);

protected:
    std::shared_ptr<upns::Checkout> m_checkout;
private:

    bool m_isInConflictMode;
    QmlRepository* m_repository;
    QString m_name;
    QStringList m_entities;

    void reloadEntities();
};

#endif
