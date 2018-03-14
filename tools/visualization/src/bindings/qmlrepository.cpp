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

#include "mapit/ui/bindings/qmlrepository.h"
#include <mapit/versioning/repositoryfactory.h>
#include <mapit/errorcodes.h>
#include <mapit/serialization/operatorlibrarymanager.h>
#include "operatorloader.h"
#include <mapit/versioning/repositoryfactorystandard.h>

QmlRepository::QmlRepository(QObject *parent)
    :QmlRepository(nullptr, nullptr)
{

}

QmlRepository::QmlRepository(std::shared_ptr<mapit::Repository> repo)
    :QmlRepository(repo, nullptr)
{

}

QmlRepository::QmlRepository(std::shared_ptr<mapit::Repository> repo, QObject *parent)
    :QObject( parent )
    , m_repository( repo )
    , m_opLoaderWorker( nullptr )
    , m_url( "." )
    , m_isLoaded( false )
{
    reload();
}

QmlRepository::~QmlRepository()
{
    if(m_opLoaderWorker)
    {
        delete m_opLoaderWorker;
    }
}
void AppendFunctionOps(QQmlListProperty<QVariant> *list, QVariant* variant)
{
    QmlRepository *repo = qobject_cast<QmlRepository*>(list->object);
    repo->m_operators.append(*variant);
}

int CountFunctionOps(QQmlListProperty<QVariant> *list)
{
    QmlRepository *repo = qobject_cast<QmlRepository*>(list->object);
    return repo->m_operators.count();
}

QVariant *AtFunctionOps(QQmlListProperty<QVariant> *list, int i)
{
    QmlRepository *repo = qobject_cast<QmlRepository*>(list->object);
    return new QVariant(&repo->m_operators.at(i)); //TODO: confirm this is not a memory leak
}

QVariantList QmlRepository::operators()
{
    return m_operators;//QQmlListProperty<QVariant>(this, nullptr, &CountFunctionOps, &AtFunctionOps);
}

void QmlRepository::reload()
{
    m_isLoadingOperators = true;
    Q_EMIT isLoadingOperatorsChanged(true);
    m_checkoutNames.clear();
    if(m_repository == nullptr) return;
    std::vector<std::string> coNames(m_repository->listCheckoutNames());
    for(std::vector<std::string>::const_iterator iter(coNames.cbegin()) ; iter != coNames.cend() ; iter++)
    {
        m_checkoutNames.append(QString::fromStdString(*iter));
    }
    m_operators.clear();
    if(m_opLoaderWorker == nullptr)
    {
        m_opLoaderWorker = new OperatorLoader();
    }
    else
    {
        disconnect(m_operatorWorkerConnection);
    }
    m_operatorWorkerConnection = connect(m_opLoaderWorker, &OperatorLoader::operatorsAdded, this, [&](QList<QVariant> result) {
        for(auto iter = result.cbegin(); iter != result.cend(); ++iter)
        {
            m_operators.append(*iter);
        }
        Q_EMIT operatorsChanged();
        m_isLoadingOperators = false;
        Q_EMIT isLoadingOperatorsChanged(false);
    }, Qt::QueuedConnection);

    reloadOperators();
    Q_EMIT isLoadedChanged(m_repository != nullptr);
    Q_EMIT checkoutNamesChanged( m_checkoutNames );
    Q_EMIT internalRepositoryChanged( this );
}

QStringList QmlRepository::listCheckoutNames() const
{
    return checkoutNames();
}

QStringList QmlRepository::checkoutNames() const
{
    return m_checkoutNames;
}

QString QmlRepository::url() const
{
    return m_url;
}

QmlTree *QmlRepository::getTree(QString oid)
{
    if(!m_repository) return nullptr;
    std::string o = oid.toStdString();
    std::shared_ptr<mapit::msgs::Tree> obj( m_repository->getTree( o ) );
    if(!obj) return nullptr;
    return new QmlTree( obj );
}

QmlEntity *QmlRepository::getEntity(QString oid)
{
    if(!m_repository) return nullptr;
    std::string o = oid.toStdString();
    std::shared_ptr<mapit::msgs::Entity> obj( m_repository->getEntity( o ) );
    if(!obj) return nullptr;
    return new QmlEntity( obj );
}

QmlCommit *QmlRepository::getCommit(QString oid)
{
    if(!m_repository) return nullptr;
    std::string o = oid.toStdString();
    std::shared_ptr<mapit::msgs::Commit> obj( m_repository->getCommit( o ) );
    if(!obj) return nullptr;
    return new QmlCommit( obj );
}

QmlBranch *QmlRepository::getBranch(QString name)
{
    if(!m_repository) return nullptr;
    std::string o = name.toStdString();
    std::shared_ptr<mapit::msgs::Branch> obj( m_repository->getBranch( o ) );
    if(!obj) return nullptr;
    return new QmlBranch( obj );
}

QString QmlRepository::typeOfObject(QString oid)
{
    if(!m_repository) return "";
    std::string o(oid.toStdString());
    mapit::msgs::MessageType mt = m_repository->typeOfObject(o);
    switch(mt)
    {
    case mapit::msgs::MessageTree:
        return"MessageTree";
    case mapit::msgs::MessageEntity:
        return"MessageEntity";
    case mapit::msgs::MessageCommit:
        return"MessageCommit";
    case mapit::msgs::MessageCheckout:
        return"MessageCheckout";
    case mapit::msgs::MessageBranch:
        return"MessageBranch";
    case mapit::msgs::MessageEntitydata:
        return"MessageEntitydata";
    case mapit::msgs::MessageEmpty:
        return"MessageEmpty";
    default:
        return"";
    }
}

QmlEntitydata *QmlRepository::getEntitydataReadOnly(QString oid)
{
    if(!m_repository) return nullptr;
    std::string o = oid.toStdString();
    std::shared_ptr<mapit::AbstractEntitydata> obj( m_repository->getEntitydataReadOnly( o ) );
    if(!obj) return nullptr;
    return new QmlEntitydata( obj, NULL );
}

void QmlRepository::reloadOperators()
{
    m_opLoaderWorker->start();
}

QmlCheckout *QmlRepository::createCheckout(QString commitIdOrBranchname, QString name)
{
    if(!m_repository) return nullptr;
    std::string o = commitIdOrBranchname.toStdString(),
                     n = name.toStdString();
    std::shared_ptr<mapit::Checkout> obj( m_repository->createCheckout( o, n ) );
    if(!obj) return nullptr;
    m_checkoutNames.append( name );
    Q_EMIT checkoutNamesChanged(m_checkoutNames);
    return new QmlCheckout( obj, this, name );
}

QmlCheckout *QmlRepository::getCheckout(QString checkoutName)
{
    if(!m_repository) return nullptr;
    std::string o = checkoutName.toStdString();
    std::shared_ptr<mapit::Checkout> obj( m_repository->getCheckout( o ) );
    if(!obj) return nullptr;
    return new QmlCheckout( obj, this, checkoutName );
}

bool QmlRepository::deleteCheckoutForced(QString checkoutName)
{
    if(!m_repository) return false;
    std::string o = checkoutName.toStdString();
    mapit::StatusCode s = m_repository->deleteCheckoutForced( o );
    return mapitIsOk( s );
}

QString QmlRepository::commit(QmlCheckout *checkout, QString msg)
{
    if(!m_repository) return nullptr;
    std::string m = msg.toStdString();
    mapit::CommitId cid = m_repository->commit( checkout->getCheckoutObj(), m );
    return QString::fromStdString(cid);
}

QList<QmlBranch *> QmlRepository::getBranches()
{
    if(!m_repository) return QList<QmlBranch *>();
    std::vector<std::shared_ptr<mapit::msgs::Branch> > b(m_repository->getBranches());
    QList<QmlBranch *> ret;
    for(std::vector<std::shared_ptr<mapit::msgs::Branch> >::const_iterator iter(b.cbegin());
        iter != b.cend();
        ++iter)
    {
        std::shared_ptr<mapit::msgs::Branch> p(*iter);
        ret.push_back(new QmlBranch( p ));
    }
    return ret;
}

QString QmlRepository::push(QmlRepository *repo)
{
    return "not yet implemented";
}

QString QmlRepository::pull(QmlRepository *repo)
{
    return "not yet implemented";
}

QString QmlRepository::parseCommitRef(QString commitRef)
{
    if(!m_repository) return nullptr;
    std::string cr = commitRef.toStdString();
    return QString::fromStdString( m_repository->parseCommitRef( cr ) );
}

QmlCheckout *QmlRepository::merge(QString mine, QString theirs, QString base)
{
    if(!m_repository) return nullptr;
    std::string m = mine.toStdString();
    std::string t = theirs.toStdString();
    std::string b = base.toStdString();
    std::shared_ptr<mapit::Checkout> co(m_repository->merge( m, t, b ));
    return new QmlCheckout( co, this );
}

QMap<QString, QString> QmlRepository::ancestors(QString commitId, QString objectId, qint32 level)
{
    //TODO: nyi
    return QMap<QString, QString>();
}

bool QmlRepository::canRead()
{
    return true;
}

bool QmlRepository::canWrite()
{
    return true;
}

std::shared_ptr<mapit::Repository> QmlRepository::getRepository()
{
    return m_repository;
}

void QmlRepository::setUrl(QString url)
{
    if (m_url == url)
        return;
    m_repository = std::shared_ptr<mapit::Repository>(mapit::RepositoryFactoryStandard::openRepositorySimple(url.toStdString(), true));

    m_url = url;
    reload();
    Q_EMIT isLoadedChanged(m_repository != nullptr);
    Q_EMIT urlChanged(url);
    Q_EMIT internalRepositoryChanged( this );
}

bool QmlRepository::isLoaded() const
{
    return m_repository != nullptr;
}

bool QmlRepository::isLoadingOperators() const
{
    return m_isLoadingOperators;
}
