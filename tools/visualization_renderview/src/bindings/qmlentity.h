#ifndef QMLENTITY_H
#define QMLENTITY_H

#include <QtCore>
#include <QJsonObject>
#include "libs/mapmanager/include/versioning/checkout.h"
#include "libs/upns_interface/services.pb.h"
#include <QQmlListProperty>

class QmlEntity : public QObject
{
    Q_OBJECT
    //Q_PROPERTY(QString id READ id WRITE setId NOTIFY idChanged)

public:
    QmlEntity(upns::Entity *obj);

public Q_SLOTS:

};

#endif
