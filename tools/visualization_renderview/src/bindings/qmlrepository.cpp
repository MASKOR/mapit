#include "qmlrepository.h"


QStringList QmlRepository::listCheckoutNames()
{
    return QStringList();
}

QmlTree *QmlRepository::getTree(QString oid)
{
    return NULL;
}

QmlEntity *QmlRepository::getEntity(QString oid)
{
    return NULL;
}

QmlCommit *QmlRepository::getCommit(QString oid)
{
    return NULL;
}

QmlCheckout *QmlRepository::getCheckout(QString name)
{
    return NULL;
}

QmlBranch *QmlRepository::getBranch(QString name)
{
    return NULL;
}

QmlCheckout *QmlRepository::createCheckout(QString commitIdOrBranchname, QString name)
{
    return NULL;
}

QmlCheckout *QmlRepository::getCheckout(QString checkoutName)
{
    return NULL;
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
    Q_EMIT confChanged(conf);
}

QString QmlRepository::conf() const
{
    return m_conf;
}

QStringList QmlRepository::checkoutNames() const
{
    return m_checkoutNames;
}
