#ifndef QMLCHECKOUT_H
#define QMLCHECKOUT_H

#include <QtCore>
#include <QJsonObject>
#include "libs/mapmanager/include/versioning/checkout.h"

class QmlCheckout : public QObject
{
    Q_OBJECT
   // Q_PROPERTY(QJsonObject config READ config WRITE setConfig NOTIFY configChanged)

public:
    Q_INVOKABLE int doOperation(const QJsonObject &desc);
};

#endif
