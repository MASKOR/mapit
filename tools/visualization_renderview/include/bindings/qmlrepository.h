#ifndef QMLREPOSITORY_H
#define QMLREPOSITORY_H

#include <QtCore>
#include "qmltree.h"
#include "qmlentity.h"
#include "qmlcommit.h"
#include "qmlcheckout.h"
#include "qmlbranch.h"
#include "qmlentitydata.h"
#include "libs/upns_interface/services.pb.h"
#include "versioning/repository.h"

class QmlCheckout;
class QmlEntitydata;
class QmlRepository : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QStringList checkoutNames READ checkoutNames NOTIFY checkoutNamesChanged)
    Q_PROPERTY(QString conf READ conf WRITE setConf NOTIFY confChanged)

public:
    QmlRepository();
    Q_INVOKABLE QmlTree* getTree(QString oid);
    Q_INVOKABLE QmlEntity* getEntity(QString oid);
    Q_INVOKABLE QmlCommit* getCommit(QString oid);
    //Q_INVOKABLE QmlCheckout* getCheckoutObj(QString name);
    Q_INVOKABLE QmlBranch* getBranch(QString name);
    Q_INVOKABLE QString typeOfObject(QString oid);
    Q_INVOKABLE QmlEntitydata* getEntityDataReadOnly(QString oid);

    /**
     * @brief checkout creates a new checkout from a commit.
     * name not existing: create new commit
     * name already existing: error (returns null).
     * @param commitId
     * @param name
     * @return
     */
    Q_INVOKABLE QmlCheckout* createCheckout(QString commitIdOrBranchname, QString name);

    Q_INVOKABLE QmlCheckout* getCheckout(QString checkoutName);

    Q_INVOKABLE bool deleteCheckoutForced(QString checkoutName);

    Q_INVOKABLE QString commit(QmlCheckout* checkout, QString msg);

    /**
     * @brief getBranches List all Branches
     * @return all Branches, names with their current HEAD commitIds.
     */
    Q_INVOKABLE QList< QmlBranch* > getBranches();

    /**
     * @brief push alls branches to <repo>
     * @param repo Other Repository with AbstractMapSerializer (maybe Network behind it?)
     * @return status
     */
    Q_INVOKABLE QString push(QmlRepository *repo);

    /**
     * @brief pull TODO: same as <repo>.push(this) ???
     * @param repo
     * @return status
     */
    Q_INVOKABLE QString pull(QmlRepository *repo);

    /**
     * @brief parseCommitRef Utility function to parse userinput like "origin/master~~^"
     * @param commitRef string
     * @return found commitId or InvalidCommitId
     */
    Q_INVOKABLE QString parseCommitRef(QString commitRef);

    /**
     * @brief merge two commits. TODO: merge vs. rebase. Based on Changed data or "replay" operations.
     * @param mine
     * @param theirs
     * @param base
     * @return A checkout in conflict mode.
     */
    Q_INVOKABLE QmlCheckout* merge(QString mine, QString theirs, QString base);

    /**
     * @brief ancestors retrieves all (or all until <level>) ancestors of an object. Note that a merged object has more parent.
     * If an object <oId1> has more than one parent <numParents> and ancestor( oId1, 1) is called, the retrieved list will have
     * <numParents> entries. This way parent siblings can be distinguishd from a hierarchy of parents.
     * @param commitId with the objectId. To put the objectId into context. An objectId might be referenced by multiple commits.
     * @param objectId
     * @param level
     * @return A List off commits with objectsIds, the complete (or up to <level>) history of an object
     */
    Q_INVOKABLE QMap< QString, QString > ancestors(QString commitId, QString objectId, qint32 level = 0);

    Q_INVOKABLE bool canRead();
    Q_INVOKABLE bool canWrite();

    QString conf() const;

    Q_INVOKABLE QStringList listCheckoutNames() const;

    QStringList checkoutNames() const
    {
        return m_checkoutNames;
    }

public Q_SLOTS:
    void setConf(QString conf);
    upns::upnsSharedPointer<upns::Repository> getRepository();

Q_SIGNALS:
    void confChanged(QString conf);

    void checkoutNamesChanged(QStringList checkoutNames);

    void internalRepositoryChanged(QmlRepository* repo);
protected:
    upns::upnsSharedPointer<upns::Repository> m_repository;

private:
    QString m_conf;
    QStringList m_checkoutNames;
};

#endif
