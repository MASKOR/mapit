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

QmlCheckoutInfo *QmlRepository::getCheckout(QString name)
{
    return NULL;
}

QmlBranch *QmlRepository::getBranch(QString name)
{
    return NULL;
}

QmlCheckout *QmlRepository::checkout(QString commitIdOrBranchname, QString name)
{
    return NULL;
}

QmlCheckout *QmlRepository::checkout(QString checkoutName)
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
