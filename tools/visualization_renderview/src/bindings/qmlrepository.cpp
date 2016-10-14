#include "qmlrepository.h"
#include "versioning/repositoryfactory.h"
#include "upns_errorcodes.h"

QmlRepository::QmlRepository()
{

}

QmlRepository::QmlRepository(QObject *parent)
    :QObject( parent )
{

}

QStringList QmlRepository::listCheckoutNames() const
{
    return checkoutNames();
}

QmlTree *QmlRepository::getTree(QString oid)
{
    if(!m_repository) return nullptr;
    upns::upnsString o = oid.toStdString();
    upns::upnsSharedPointer<upns::Tree> obj( m_repository->getTree( o ) );
    if(!obj) return nullptr;
    return new QmlTree( obj );
}

QmlEntity *QmlRepository::getEntity(QString oid)
{
    if(!m_repository) return nullptr;
    upns::upnsString o = oid.toStdString();
    upns::upnsSharedPointer<upns::Entity> obj( m_repository->getEntity( o ) );
    if(!obj) return nullptr;
    return new QmlEntity( obj );
}

QmlCommit *QmlRepository::getCommit(QString oid)
{
    if(!m_repository) return nullptr;
    upns::upnsString o = oid.toStdString();
    upns::upnsSharedPointer<upns::Commit> obj( m_repository->getCommit( o ) );
    if(!obj) return nullptr;
    return new QmlCommit( obj );
}

QmlBranch *QmlRepository::getBranch(QString name)
{
    if(!m_repository) return nullptr;
    upns::upnsString o = name.toStdString();
    upns::upnsSharedPointer<upns::Branch> obj( m_repository->getBranch( o ) );
    if(!obj) return nullptr;
    return new QmlBranch( obj );
}

QString QmlRepository::typeOfObject(QString oid)
{
    if(!m_repository) return "";
    std::string o(oid.toStdString());
    upns::MessageType mt = m_repository->typeOfObject(o);
    switch(mt)
    {
    case upns::MessageTree:
        return"MessageTree";
    case upns::MessageEntity:
        return"MessageEntity";
    case upns::MessageCommit:
        return"MessageCommit";
    case upns::MessageCheckout:
        return"MessageCheckout";
    case upns::MessageBranch:
        return"MessageBranch";
    case upns::MessageEntityData:
        return"MessageEntityData";
    case upns::MessageEmpty:
        return"MessageEmpty";
    default:
        return"";
    }
}

QmlEntitydata *QmlRepository::getEntityDataReadOnly(QString oid)
{
    if(!m_repository) return nullptr;
    upns::upnsString o = oid.toStdString();
    upns::upnsSharedPointer<upns::AbstractEntityData> obj( m_repository->getEntityDataReadOnly( o ) );
    if(!obj) return nullptr;
    return new QmlEntitydata( obj, NULL );
}

QmlCheckout *QmlRepository::createCheckout(QString commitIdOrBranchname, QString name)
{
    if(!m_repository) return nullptr;
    upns::upnsString o = commitIdOrBranchname.toStdString(),
                     n = name.toStdString();
    upns::upnsSharedPointer<upns::Checkout> obj( m_repository->createCheckout( o, n ) );
    if(!obj) return nullptr;
    m_checkoutNames.append( name );
    Q_EMIT checkoutNamesChanged(m_checkoutNames);
    return new QmlCheckout( obj, this, name );
}

QmlCheckout *QmlRepository::getCheckout(QString checkoutName)
{
    if(!m_repository) return nullptr;
    upns::upnsString o = checkoutName.toStdString();
    upns::upnsSharedPointer<upns::Checkout> obj( m_repository->getCheckout( o ) );
    if(!obj) return nullptr;
    return new QmlCheckout( obj, this, checkoutName );
}

bool QmlRepository::deleteCheckoutForced(QString checkoutName)
{
    if(!m_repository) return false;
    upns::upnsString o = checkoutName.toStdString();
    upns::StatusCode s = m_repository->deleteCheckoutForced( o );
    return upnsIsOk( s );
}

QString QmlRepository::commit(QmlCheckout *checkout, QString msg)
{
    if(!m_repository) return nullptr;
    upns::upnsString m = msg.toStdString();
    upns::CommitId cid = m_repository->commit( checkout->getCheckoutObj(), m );
    return QString::fromStdString(cid);
}

QList<QmlBranch *> QmlRepository::getBranches()
{
    if(!m_repository) return QList<QmlBranch *>();
    upns::upnsVec<upns::upnsSharedPointer<upns::Branch> > b(m_repository->getBranches());
    QList<QmlBranch *> ret;
    for(upns::upnsVec<upns::upnsSharedPointer<upns::Branch> >::const_iterator iter(b.cbegin());
        iter != b.cend();
        ++iter)
    {
        upns::upnsSharedPointer<upns::Branch> p(*iter);
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
    upns::upnsString cr = commitRef.toStdString();
    return QString::fromStdString( m_repository->parseCommitRef( cr ) );
}

QmlCheckout *QmlRepository::merge(QString mine, QString theirs, QString base)
{
    if(!m_repository) return nullptr;
    upns::upnsString m = mine.toStdString();
    upns::upnsString t = theirs.toStdString();
    upns::upnsString b = base.toStdString();
    upns::upnsSharedPointer<upns::Checkout> co(m_repository->merge( m, t, b ));
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

void QmlRepository::setConf(QString conf)
{
    if (m_conf == conf)
        return;
    m_conf = conf;
    m_repository = upns::upnsSharedPointer<upns::Repository>(upns::RepositoryFactory::openLocalRepository(conf.toStdString()));
    m_checkoutNames.clear();
    upns::upnsVec<upns::upnsString> coNames(m_repository->listCheckoutNames());
    for(upns::upnsVec<upns::upnsString>::const_iterator iter(coNames.cbegin()) ; iter != coNames.cend() ; iter++)
    {
        m_checkoutNames.append(QString::fromStdString(*iter));
    }
    Q_EMIT checkoutNamesChanged( m_checkoutNames );
    Q_EMIT confChanged( conf );
    Q_EMIT internalRepositoryChanged( this );
}

upns::upnsSharedPointer<upns::Repository> QmlRepository::getRepository()
{
    return m_repository;
}

QString QmlRepository::conf() const
{
    return m_conf;
}
