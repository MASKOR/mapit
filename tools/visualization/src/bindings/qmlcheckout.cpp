#include "upns/ui/bindings/qmlcheckout.h"
#include <upns/errorcodes.h>

QmlCheckout::QmlCheckout()
    :m_checkout( NULL ),
     m_repository( NULL )
{

}

// If name is not given, this must be a result of a merge.
QmlCheckout::QmlCheckout(std::shared_ptr<upns::Checkout> &co, QmlRepository* repo, QString name)
    :m_checkout( co ),
     m_repository( repo ),
     m_name( name )
{
    if(m_repository)
    {
        connect(m_repository, &QmlRepository::internalRepositoryChanged, this, &QmlCheckout::setRepository);
    }
}

QString QmlCheckout::doOperation(QString operatorname, const QJsonObject &desc)
{
    if(!m_checkout) return "not initialized, too early";
    QJsonDocument doc(desc);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    mapit::msgs::OperationDescription descript;
    descript.set_operatorname(operatorname.toStdString());
    descript.set_params(strJson.toStdString());
    upns::OperationResult res = m_checkout->doOperation(descript);
    if(upnsIsOk(res.first)) return "error";
    //TODO Q_EMIT something changed()
    // TODO: wrap operation result.
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

QmlRepository *QmlCheckout::repository() const
{
    return m_repository;
}

QString QmlCheckout::name() const
{
    return m_name;
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
    if(!m_name.isEmpty())
    {
        m_checkout = m_repository->getRepository()->getCheckout(m_name.toStdString());
        Q_EMIT intenalCheckoutChanged( this );
    }
    //TODO: there might be some rare cases, where this is set intentionally to "".
    //Important: on merge it is "" and should not be overwritten.
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
        Q_EMIT intenalCheckoutChanged( this );
    }

    Q_EMIT nameChanged(name);
}
