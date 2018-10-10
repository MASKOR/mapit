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

#ifndef QmlWorkspace_H
#define QmlWorkspace_H

#include <QtCore>

#include "qmltree.h"
#include "qmlentity.h"
#include "qmlcommit.h"
#include "qmlworkspace.h"
#include "qmlbranch.h"
#include "qmlentitydata.h"
#include "qmlrepository.h"
#include <mapit/versioning/workspace.h>
#include <mapit/msgs/services.pb.h>
#include <QtCore/QJsonObject>

class QmlRepository;
class OperationExecutor;

class QmlWorkspace : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool isInConflictMode READ isInConflictMode NOTIFY isInConflictModeChanged)
    Q_PROPERTY(QmlRepository* repository READ repository WRITE setRepository NOTIFY repositoryChanged)
    Q_PROPERTY(QString name READ name WRITE setName NOTIFY nameChanged)
    Q_PROPERTY(QStringList entities READ entities NOTIFY entitiesChanged)
    Q_PROPERTY(bool isBusyExecuting READ isBusyExecuting NOTIFY isBusyExecutingChanged)
    Q_PROPERTY(int lastOperationStatus READ lastOperationStatus NOTIFY lastOperationStatusChanged)
public:
    QmlWorkspace();
    QmlWorkspace(std::shared_ptr<mapit::Workspace> &workspace, QmlRepository* repo = NULL, QString name = "" );
    ~QmlWorkspace();

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
    std::shared_ptr<mapit::Workspace> getWorkspaceObj();
    QmlRepository* repository() const;
    QString name() const;
    QStringList entities() const;

    //TODO: Put this to a another class. This introduces dependency
    //      to tfs from workspace (core->entitytype).

    Q_INVOKABLE QStringList getFrameIds();
    bool isBusyExecuting() const;

    int lastOperationStatus() const;

public Q_SLOTS:
    void setRepository(QmlRepository* repository);
    void setName(QString name);
    void operationExecuted(int result);
Q_SIGNALS:
    void isInConflictModeChanged(bool isInConflictMode);
    void repositoryChanged(QmlRepository* repository);
    void nameChanged(QString name);
    void internalWorkspaceChanged(QmlWorkspace *co);
    void entitiesChanged(QStringList entities);
    void isBusyExecutingChanged(bool isBusyExecuting);

    void lastOperationStatusChanged(int lastOperationStatus);

protected:
    std::shared_ptr<mapit::Workspace> m_workspace;
private:

    QmlRepository* m_repository;
    bool m_isInConflictMode;
    bool m_isBusyExecuting;
    OperationExecutor *m_executor;
    QString m_name;
    QStringList m_entities;
    QSet<QString> frameIdSet;

    void reloadEntities();
    int m_lastOperationStatus;
};

#endif
