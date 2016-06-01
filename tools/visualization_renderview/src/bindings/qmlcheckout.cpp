#include "qmlcheckout.h"


QmlCheckout::QmlCheckout()
    :m_checkout( nullptr )
{

}

QmlCheckout::QmlCheckout(upns::upnsSharedPointer<upns::Checkout> &co)
    :m_checkout( co )
{

}

QString QmlCheckout::doOperation(QString operatorname, const QJsonObject &desc)
{
    return "";
}

void QmlCheckout::setConflictSolved(QString path, QString oid)
{
    upns::upnsString p = path.toStdString(),
                     o = oid.toStdString();
    m_checkout->setConflictSolved(p, o);
}

QmlTree *QmlCheckout::getRoot()
{
    upns::upnsSharedPointer<upns::Tree> tree(m_checkout->getRoot());
    return new QmlTree(tree);
}

QmlTree *QmlCheckout::getTreeConflict(QString objectId)
{
    upns::upnsString p = objectId.toStdString();
    upns::upnsSharedPointer<upns::Tree> tree(m_checkout->getTreeConflict(p));
    return new QmlTree(tree);
}

QmlEntity *QmlCheckout::getEntityConflict(QString objectId)
{
    upns::upnsString p = objectId.toStdString();
    upns::upnsSharedPointer<upns::Entity> ent(m_checkout->getEntityConflict(p));
    return new QmlEntity(ent);
}

QmlTree *QmlCheckout::getTree(QString path)
{
    upns::upnsString p = path.toStdString();
    upns::upnsSharedPointer<upns::Tree> tree(m_checkout->getTree(p));
    return new QmlTree(tree);
}

QmlEntity *QmlCheckout::getEntity(QString path)
{
    upns::upnsString p = path.toStdString();
    upns::upnsSharedPointer<upns::Entity> ent(m_checkout->getEntity(p));
    return new QmlEntity(ent);
}

QmlBranch *QmlCheckout::getParentBranch()
{
    upns::upnsSharedPointer<upns::Branch> br(m_checkout->getParentBranch());
    return new QmlBranch(br);
}

QStringList QmlCheckout::getParentCommitIds()
{
    upns::upnsVec<upns::CommitId> ids(m_checkout->getParentCommitIds());
    QStringList allIds;
    std::for_each(ids.begin(), ids.end(), [&allIds](const upns::CommitId &id)
    {
        allIds.append(QString::fromStdString(id));
    });

    return allIds;
}

QmlEntitydata *QmlCheckout::getEntitydataReadOnly(QString path)
{
    upns::upnsString p = path.toStdString();
    upns::upnsSharedPointer<upns::AbstractEntityData> ent(m_checkout->getEntitydataReadOnly(p));
    return new QmlEntitydata(ent);
}

QmlEntitydata *QmlCheckout::getEntitydataReadOnlyConflict(QString entityId)
{
    upns::upnsString oid = entityId.toStdString();
    upns::upnsSharedPointer<upns::AbstractEntityData> ent(m_checkout->getEntitydataReadOnlyConflict(oid));
    return new QmlEntitydata(ent);
}

bool QmlCheckout::isInConflictMode() const
{
    return m_isInConflictMode;
}
