#ifndef QMLREPOSITORY_H
#define QMLREPOSITORY_H

#include <QtCore>
#include "qmltree.h"
#include "qmlentity.h"
#include "qmlcommit.h"
#include "qmlcheckout.h"
#include "qmlbranch.h"
#include "qmlentitydata.h"
#include <mapit/msgs/services.pb.h>
#include <upns/versioning/repository.h>

class OperatorLoader;
class QmlCheckout;
class QmlEntitydata;
class QmlRepository : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QStringList checkoutNames READ checkoutNames NOTIFY checkoutNamesChanged)

    Q_PROPERTY(QVariantList operators READ operators NOTIFY operatorsChanged)
public:
    QmlRepository(std::shared_ptr<upns::Repository> repo);
    QmlRepository(std::shared_ptr<upns::Repository> repo, QObject *parent);
    ~QmlRepository();
    QVariantList operators();
    Q_INVOKABLE QmlTree* getTree(QString oid);
    Q_INVOKABLE QmlEntity* getEntity(QString oid);
    Q_INVOKABLE QmlCommit* getCommit(QString oid);
    //Q_INVOKABLE QmlCheckout* getCheckoutObj(QString name);
    Q_INVOKABLE QmlBranch* getBranch(QString name);
    Q_INVOKABLE QString typeOfObject(QString oid);
    Q_INVOKABLE QmlEntitydata* getEntitydataReadOnly(QString oid);

    //TODO: this might find a better place in the future. It is not part of repository.
    Q_INVOKABLE void reloadOperators();

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
     * @param repo Other Repository with AbstractSerializer (maybe Network behind it?)
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

    Q_INVOKABLE QStringList listCheckoutNames() const;

    QStringList checkoutNames() const
    {
        return m_checkoutNames;
    }

    QVariantList m_operators;
public Q_SLOTS:
    std::shared_ptr<upns::Repository> getRepository();

Q_SIGNALS:
    void checkoutNamesChanged(QStringList checkoutNames);

    void internalRepositoryChanged(QmlRepository* repo);

    void operatorsChanged();
protected:
    std::shared_ptr<upns::Repository> m_repository;

private:
    QStringList m_checkoutNames;
    OperatorLoader *m_opLoaderWorker;
};

#endif
