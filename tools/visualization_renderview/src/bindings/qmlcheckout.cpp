#include "qmlcheckout.h"


QString QmlCheckout::doOperation(QString operatorname, const QJsonObject &desc)
{
    return "";
}

void QmlCheckout::setConflictSolved(QString path, QString oid)
{
    m_checkout->setConflictSolved(path, oid);
}

QmlTree *QmlCheckout::getRoot()
{

}

QmlTree *QmlCheckout::getTreeConflict(QString objectId)
{

}

QmlEntity *QmlCheckout::getEntityConflict(QString objectId)
{

}

QmlTree *QmlCheckout::getTree(QString path)
{

}

QmlEntity *QmlCheckout::getEntity(QString path)
{

}

QmlBranch *QmlCheckout::getParentBranch()
{

}

QString QmlCheckout::getParentCommitIds()
{

}

QmlEntitydata *QmlCheckout::getEntityDataReadOnly(QString entityId)
{

}

QmlEntitydata *QmlCheckout::getEntityDataReadOnlyConflict(QString entityId)
{

}

bool QmlCheckout::isInConflictMode() const
{
    return m_isInConflictMode;
}
