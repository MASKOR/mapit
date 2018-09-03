/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "mapit/ui/bindings/qmlworkspace.h"
#include <mapit/errorcodes.h>
#include <mapit/depthfirstsearch.h>
#include <mapit/layertypes/tflayer.h> //< TODO: remove this dependecy and put dependet code to it to new "QmlTransformLayer"
#include <mapit/layertypes/tflayer/tf2/buffer_core.h>
#include <mapit/layertypes/tflayer/tf2/exceptions.h>
#include <QStringList>
#include <QSet>
#include "operationexecutor.h"

QmlWorkspace::QmlWorkspace()
    : m_workspace( NULL )
    , m_repository( NULL )
    , m_isInConflictMode( false )
    , m_isBusyExecuting( false )
    , m_executor( nullptr )
{
}

// If name is not given, this must be a result of a merge.
QmlWorkspace::QmlWorkspace(std::shared_ptr<mapit::Workspace> &workspace, QmlRepository* repo, QString name)
    : m_workspace( workspace )
    , m_repository( repo )
    , m_isInConflictMode( false )
    , m_isBusyExecuting( false )
    , m_executor( nullptr )
    , m_name( name )
{
    if(m_repository)
    {
        connect(m_repository, &QmlRepository::internalRepositoryChanged, this, &QmlWorkspace::setRepository);
    }
    reloadEntities();
}

QmlWorkspace::~QmlWorkspace()
{
    if(m_executor) delete m_executor;
}

QString QmlWorkspace::doOperation(QString operatorname, const QJsonObject &desc)
{
    if(!m_workspace) return "not initialized, too early";

    if(isBusyExecuting()) return "can not execute, currently busy executing";
    if(m_executor) delete m_executor;

    QJsonDocument doc(desc);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    mapit::msgs::OperationDescription descript;
    descript.mutable_operator_()->set_operatorname(operatorname.toStdString());
    descript.set_params(strJson.toStdString());

    m_executor = new OperationExecutor(this, m_workspace, descript);

    connect(m_executor, &OperationExecutor::operationExecuted, this, &QmlWorkspace::operationExecuted);

    m_isBusyExecuting = true;
    Q_EMIT isBusyExecutingChanged(m_isBusyExecuting);

    m_executor->start();
//    mapit::OperationResult res = m_workspace->doOperation(descript);
//    reloadEntities();
//    Q_EMIT internalWorkspaceChanged(this);
//    //TODO Q_EMIT something changed()
//    // TODO: wrap operation result.
//    if(mapitIsOk(res.first)) return "error";
    return "";
}

void QmlWorkspace::setConflictSolved(QString path, QString oid)
{
    //TODO: how to handle error?
    if(!m_workspace) return;
    std::string p = path.toStdString(),
                     o = oid.toStdString();
    m_workspace->setConflictSolved(p, o);
}

QmlTree *QmlWorkspace::getRoot()
{
    if(!m_workspace) return new QmlTree(this);
    std::shared_ptr<mapit::msgs::Tree> tree(m_workspace->getRoot());
    return new QmlTree(tree);
}

QmlTree *QmlWorkspace::getTreeConflict(QString objectId)
{
    if(!m_workspace) return new QmlTree(this);
    std::string p = objectId.toStdString();
    std::shared_ptr<mapit::msgs::Tree> tree(m_workspace->getTreeConflict(p));
    return new QmlTree(tree);
}

QmlEntity *QmlWorkspace::getEntityConflict(QString objectId)
{
    if(!m_workspace) return new QmlEntity(this);
    std::string p = objectId.toStdString();
    std::shared_ptr<mapit::msgs::Entity> ent(m_workspace->getEntityConflict(p));
    return new QmlEntity(ent);
}

QmlTree *QmlWorkspace::getTree(QString path)
{
    if(!m_workspace) return new QmlTree(this);
    std::string p = path.toStdString();
    std::shared_ptr<mapit::msgs::Tree> tree(m_workspace->getTree(p));
    return new QmlTree(tree);
}

QmlEntity *QmlWorkspace::getEntity(QString path)
{
    if(!m_workspace) return new QmlEntity(this);
    if(path.isEmpty()) return new QmlEntity(this);
    std::string p = path.toStdString();
    std::shared_ptr<mapit::msgs::Entity> ent(m_workspace->getEntity(p));
    return new QmlEntity(ent);
}

QmlBranch *QmlWorkspace::getParentBranch()
{
    if(!m_workspace) return new QmlBranch(this);
    std::shared_ptr<mapit::msgs::Branch> br(m_workspace->getParentBranch());
    return new QmlBranch(br);
}

QStringList QmlWorkspace::getParentCommitIds()
{
    if(!m_workspace) return QStringList();
    std::vector<mapit::CommitId> ids(m_workspace->getParentCommitIds());
    QStringList allIds;
    std::for_each(ids.begin(), ids.end(), [&allIds](const mapit::CommitId &id)
    {
        allIds.append(QString::fromStdString(id));
    });

    return allIds;
}

QmlEntitydata *QmlWorkspace::getEntitydataReadOnly(QString path)
{
    if(!m_workspace) return new QmlEntitydata(this);
    std::string p = path.toStdString();
    std::shared_ptr<mapit::AbstractEntitydata> ent(m_workspace->getEntitydataReadOnly(p));
    return new QmlEntitydata(ent, this, path);
}

QmlEntitydata *QmlWorkspace::getEntitydataReadOnlyConflict(QString entityId)
{
    if(!m_workspace) return new QmlEntitydata(this);
    std::string oid = entityId.toStdString();
    std::shared_ptr<mapit::AbstractEntitydata> ent(m_workspace->getEntitydataReadOnlyConflict(oid));
    return new QmlEntitydata(ent, this);
}

bool QmlWorkspace::isInConflictMode() const
{
    return m_isInConflictMode;
}

std::shared_ptr<mapit::Workspace> QmlWorkspace::getWorkspaceObj() { return m_workspace; }

QmlRepository *QmlWorkspace::repository() const
{
    return m_repository;
}

QString QmlWorkspace::name() const
{
    return m_name;
}

QStringList QmlWorkspace::entities() const
{
    return m_entities;
}

QStringList QmlWorkspace::getFrameIds()
{
    if(this->m_workspace == nullptr) return QStringList();
    if ( frameIdSet.empty() ) {
        mapit::StatusCode s = this->m_workspace->depthFirstSearch(
            depthFirstSearchWorkspaceAll(Tree), depthFirstSearchWorkspaceAll(Tree),
            [&](std::shared_ptr<Entity> obj, const ObjectReference& ref, const mapit::Path &path)
            {
                // get the stream to write into a file
                assert(obj);
                frameIdSet.insert(QString::fromStdString(obj->frame_id()));

                if(strcmp(obj->type().c_str(), TfEntitydata::TYPENAME()) == 0)
                {
                    std::shared_ptr<mapit::AbstractEntitydata> ed = this->m_workspace->getEntitydataReadOnly( path );
                    if( ed == nullptr ) return true;
                    std::shared_ptr<TfEntitydata> tfEd = std::dynamic_pointer_cast<TfEntitydata>( ed );
                    if( tfEd == nullptr ) return true;
                    std::shared_ptr<mapit::tf::store::TransformStampedList> tfStore = tfEd->getData();
                    if(tfStore == nullptr ) return true;
                    frameIdSet.insert(QString::fromStdString( tfStore->get_child_frame_id() ));
                }
                return true;
            },
            depthFirstSearchWorkspaceAll(Entity));
        if( !mapitIsOk(s) ) {
            log_warn("getFrameIds did not succeed for UI");
        }
    }

    return frameIdSet.toList();
}

bool QmlWorkspace::isBusyExecuting() const
{
    return m_isBusyExecuting;
}

int QmlWorkspace::lastOperationStatus() const
{
    return m_lastOperationStatus;
}

void QmlWorkspace::setRepository(QmlRepository *repository)
{
    if (m_repository != repository)
    {
        if(m_repository)
        {
            disconnect(m_repository, &QmlRepository::internalRepositoryChanged, this, &QmlWorkspace::setRepository);
        }
        m_repository = repository;
        if(m_repository)
        {
            connect(m_repository, &QmlRepository::internalRepositoryChanged, this, &QmlWorkspace::setRepository);
        }
        Q_EMIT repositoryChanged(repository);
    }
    if(!m_name.isEmpty() && m_repository->getRepository())
    {
        m_workspace = m_repository->getRepository()->getWorkspace(m_name.toStdString());
        reloadEntities();
        Q_EMIT internalWorkspaceChanged( this );
    }
    //TODO: there might be some rare cases, where this is set intentionally to "".
    //Important: on merge workspace name might be "" and should not be overwritten.
//    else
//    {
//        m_workspace = NULL;
//    }
}

void QmlWorkspace::setName(QString name)
{
    if (m_name == name)
        return;

    m_name = name;
    if(m_repository)
    {
        m_workspace = m_repository->getRepository()->getWorkspace(m_name.toStdString());
        reloadEntities();
        Q_EMIT internalWorkspaceChanged( this );
    }

    Q_EMIT nameChanged(name);
}

void QmlWorkspace::operationExecuted(int result)
{
    m_isBusyExecuting = false;
    m_lastOperationStatus = result;
    reloadEntities();
    Q_EMIT internalWorkspaceChanged(this);
    Q_EMIT isBusyExecutingChanged(m_isBusyExecuting);
    Q_EMIT lastOperationStatusChanged(result);
}

void QmlWorkspace::reloadEntities()
{
    frameIdSet.clear();
    m_entities.clear();
    if(m_workspace)
    {
        mapit::StatusCode s = m_workspace->depthFirstSearch(
            [&](std::shared_ptr<Tree> obj, const ObjectReference& ref, const mapit::Path &path)
            {
                return true;
            }, [&](std::shared_ptr<Tree> obj, const ObjectReference& ref, const mapit::Path &path)
            {
                return true;
            },
            [&](std::shared_ptr<Entity> obj, const ObjectReference& ref, const mapit::Path &path)
            {
                m_entities.append(QString::fromStdString(path));
                return true;
            },
            [&](std::shared_ptr<Entity> obj, const ObjectReference& ref, const mapit::Path &path)
            {
                return true;
            });
    }
    Q_EMIT entitiesChanged(m_entities);
}
