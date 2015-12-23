#ifndef QMLENTITY_H
#define QMLENTITY_H

#include <QtCore>
#include <QJsonObject>
#include "libs/mapmanager/src/mapmanager.h"
#include "libs/upns_interface/services.pb.h"
#include <QQmlListProperty>

class QmlEntity : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString id READ id WRITE setId NOTIFY idChanged)

public:
    QmlEntity(upns::Entity *obj);

    QString id() const
    {
        return QString::number(m_entity->id());
    }

public Q_SLOTS:


    void setId(QString id)
    {
        ::google::protobuf::int64 ident = id.toULongLong();
        if (m_entity->id() == ident)
            return;

        m_entity->set_id(ident);
        Q_EMIT idChanged(id);
    }

Q_SIGNALS:
    void idChanged(QString id);

private:
    upns::Entity *m_entity;
};

#endif
