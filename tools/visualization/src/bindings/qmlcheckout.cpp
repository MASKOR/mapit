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

#include "upns/ui/bindings/qmlcheckout.h"
#include <upns/errorcodes.h>
#include "upns/depthfirstsearch.h"
#include <upns/layertypes/tflayer.h> //< TODO: remove this dependecy and put dependet code to it to new "QmlTransformLayer"
#include <upns/layertypes/tflayer/tf2/buffer_core.h>
#include <upns/layertypes/tflayer/tf2/exceptions.h>
#include <QStringList>
#include <QSet>
#include "operationexecutor.h"

QmlCheckout::QmlCheckout()
    : m_checkout( NULL )
    , m_repository( NULL )
    , m_isInConflictMode( false )
    , m_isBusyExecuting( false )
    , m_executor( nullptr )
{
}

// If name is not given, this must be a result of a merge.
QmlCheckout::QmlCheckout(std::shared_ptr<upns::Checkout> &co, QmlRepository* repo, QString name)
    : m_checkout( co )
    , m_repository( repo )
    , m_isInConflictMode( false )
    , m_isBusyExecuting( false )
    , m_executor( nullptr )
    , m_name( name )
{
    if(m_repository)
    {
        connect(m_repository, &QmlRepository::internalRepositoryChanged, this, &QmlCheckout::setRepository);
    }
    reloadEntities();
}

QmlCheckout::~QmlCheckout()
{
    if(m_executor) delete m_executor;
}

QString QmlCheckout::doOperation(QString operatorname, const QJsonObject &desc)
{
    if(!m_checkout) return "not initialized, too early";

    if(isBusyExecuting()) return "can not execute, currently busy executing";
    if(m_executor) delete m_executor;

    QJsonDocument doc(desc);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    mapit::msgs::OperationDescription descript;
    descript.mutable_operator_()->set_operatorname(operatorname.toStdString());
    descript.set_params(strJson.toStdString());

    m_executor = new OperationExecutor(this, m_checkout, descript);

    connect(m_executor, &OperationExecutor::operationExecuted, this, &QmlCheckout::operationExecuted);

    m_isBusyExecuting = true;
    Q_EMIT isBusyExecutingChanged(m_isBusyExecuting);

    m_executor->start();
//    upns::OperationResult res = m_checkout->doOperation(descript);
//    reloadEntities();
//    Q_EMIT internalCheckoutChanged(this);
//    //TODO Q_EMIT something changed()
//    // TODO: wrap operation result.
//    if(upnsIsOk(res.first)) return "error";
    return "";
}

void QmlCheckout::setConflictSolved(QString path, QString oid)
{
    //TODO: how to handle error?
    if(!m_checkout) return;
    std::string p = path.toStdString(),
                     o = oid.toStdString();
    m_checkout->setConflictSolved(p, o);
}

QmlTree *QmlCheckout::getRoot()
{
    if(!m_checkout) return new QmlTree(this);
    std::shared_ptr<mapit::msgs::Tree> tree(m_checkout->getRoot());
    return new QmlTree(tree);
}

QmlTree *QmlCheckout::getTreeConflict(QString objectId)
{
    if(!m_checkout) return new QmlTree(this);
    std::string p = objectId.toStdString();
    std::shared_ptr<mapit::msgs::Tree> tree(m_checkout->getTreeConflict(p));
    return new QmlTree(tree);
}

QmlEntity *QmlCheckout::getEntityConflict(QString objectId)
{
    if(!m_checkout) return new QmlEntity(this);
    std::string p = objectId.toStdString();
    std::shared_ptr<mapit::msgs::Entity> ent(m_checkout->getEntityConflict(p));
    return new QmlEntity(ent);
}

QmlTree *QmlCheckout::getTree(QString path)
{
    if(!m_checkout) return new QmlTree(this);
    std::string p = path.toStdString();
    std::shared_ptr<mapit::msgs::Tree> tree(m_checkout->getTree(p));
    return new QmlTree(tree);
}

QmlEntity *QmlCheckout::getEntity(QString path)
{
    if(!m_checkout) return new QmlEntity(this);
    if(path.isEmpty()) return new QmlEntity(this);
    std::string p = path.toStdString();
    std::shared_ptr<mapit::msgs::Entity> ent(m_checkout->getEntity(p));
    return new QmlEntity(ent);
}

QmlBranch *QmlCheckout::getParentBranch()
{
    if(!m_checkout) return new QmlBranch(this);
    std::shared_ptr<mapit::msgs::Branch> br(m_checkout->getParentBranch());
    return new QmlBranch(br);
}

QStringList QmlCheckout::getParentCommitIds()
{
    if(!m_checkout) return QStringList();
    std::vector<upns::CommitId> ids(m_checkout->getParentCommitIds());
    QStringList allIds;
    std::for_each(ids.begin(), ids.end(), [&allIds](const upns::CommitId &id)
    {
        allIds.append(QString::fromStdString(id));
    });

    return allIds;
}

QmlEntitydata *QmlCheckout::getEntitydataReadOnly(QString path)
{
    if(!m_checkout) return new QmlEntitydata(this);
    std::string p = path.toStdString();
    std::shared_ptr<upns::AbstractEntitydata> ent(m_checkout->getEntitydataReadOnly(p));
    return new QmlEntitydata(ent, this, path);
}

QmlEntitydata *QmlCheckout::getEntitydataReadOnlyConflict(QString entityId)
{
    if(!m_checkout) return new QmlEntitydata(this);
    std::string oid = entityId.toStdString();
    std::shared_ptr<upns::AbstractEntitydata> ent(m_checkout->getEntitydataReadOnlyConflict(oid));
    return new QmlEntitydata(ent, this);
}

bool QmlCheckout::isInConflictMode() const
{
    return m_isInConflictMode;
}

std::shared_ptr<Checkout> QmlCheckout::getCheckoutObj() { return m_checkout; }

QmlRepository *QmlCheckout::repository() const
{
    return m_repository;
}

QString QmlCheckout::name() const
{
    return m_name;
}

QStringList QmlCheckout::entities() const
{
    return m_entities;
}

QStringList QmlCheckout::getFrameIds()
{
    if(this->m_checkout == nullptr) return QStringList();
    QSet<QString> frameIdSet;

    upns::StatusCode s = this->m_checkout->depthFirstSearch(
        depthFirstSearchAll(Tree), depthFirstSearchAll(Tree),
        [&](std::shared_ptr<Entity> obj, const ObjectReference& ref, const upns::Path &path)
        {
            // get the stream to write into a file
            std::shared_ptr<upns::AbstractEntitydata> ed = this->m_checkout->getEntitydataReadOnly(path);

            std::shared_ptr<mapit::msgs::Entity> ent = this->m_checkout->getEntity( path );
            assert(ent);
            frameIdSet.insert(QString::fromStdString(ent->frame_id()));

            if(ed && strcmp(ed->type(), TfEntitydata::TYPENAME()) == 0)
            {
                std::shared_ptr<upns::AbstractEntitydata> ed = this->m_checkout->getEntitydataReadOnly( path );
                if( ed == nullptr ) return true;
                std::shared_ptr<TfEntitydata> tfEd = std::dynamic_pointer_cast<TfEntitydata>( ed );
                if( tfEd == nullptr ) return true;
                std::shared_ptr<tf::store::TransformStampedList> tfStore = tfEd->getData();
                if(tfStore == nullptr ) return true;
                frameIdSet.insert(QString::fromStdString( tfStore->get_child_frame_id() ));
            }
            return true;
        },
        depthFirstSearchAll(Entity));
    if( !upnsIsOk(s) ) {
        log_warn("getFrameIds did not succeed for UI");
    }
    return frameIdSet.toList();
}

bool QmlCheckout::isBusyExecuting() const
{
    return m_isBusyExecuting;
}

int QmlCheckout::lastOperationStatus() const
{
    return m_lastOperationStatus;
}

void QmlCheckout::setRepository(QmlRepository *repository)
{
    if (m_repository != repository)
    {
        if(m_repository)
        {
            disconnect(m_repository, &QmlRepository::internalRepositoryChanged, this, &QmlCheckout::setRepository);
        }
        m_repository = repository;
        if(m_repository)
        {
            connect(m_repository, &QmlRepository::internalRepositoryChanged, this, &QmlCheckout::setRepository);
        }
        Q_EMIT repositoryChanged(repository);
    }
    if(!m_name.isEmpty() && m_repository->getRepository())
    {
        m_checkout = m_repository->getRepository()->getCheckout(m_name.toStdString());
        reloadEntities();
        Q_EMIT internalCheckoutChanged( this );
    }
    //TODO: there might be some rare cases, where this is set intentionally to "".
    //Important: on merge checkout name might be "" and should not be overwritten.
//    else
//    {
//        m_checkout = NULL;
//    }
}

void QmlCheckout::setName(QString name)
{
    if (m_name == name)
        return;

    m_name = name;
    if(m_repository)
    {
        m_checkout = m_repository->getRepository()->getCheckout(m_name.toStdString());
        reloadEntities();
        Q_EMIT internalCheckoutChanged( this );
    }

    Q_EMIT nameChanged(name);
}

void QmlCheckout::operationExecuted(int result)
{
    m_isBusyExecuting = false;
    m_lastOperationStatus = result;
    reloadEntities();
    Q_EMIT internalCheckoutChanged(this);
    Q_EMIT isBusyExecutingChanged(m_isBusyExecuting);
    Q_EMIT lastOperationStatusChanged(result);
}

void QmlCheckout::reloadEntities()
{
    m_entities.clear();
    if(m_checkout)
    {
        upns::StatusCode s = m_checkout->depthFirstSearch(
            [&](std::shared_ptr<Tree> obj, const ObjectReference& ref, const upns::Path &path)
            {
                return true;
            }, [&](std::shared_ptr<Tree> obj, const ObjectReference& ref, const upns::Path &path)
            {
                return true;
            },
            [&](std::shared_ptr<Entity> obj, const ObjectReference& ref, const upns::Path &path)
            {
                m_entities.append(QString::fromStdString(path));
                return true;
            },
            [&](std::shared_ptr<Entity> obj, const ObjectReference& ref, const upns::Path &path)
            {
                return true;
            });
    }
    Q_EMIT entitiesChanged(m_entities);
}
