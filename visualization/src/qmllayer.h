#ifndef QMLLAYER_H
#define QMLLAYER_H

#include <QtCore>
#include <QJsonObject>
#include "mapmanager/src/mapmanager.h"
#include "upns_interface/services.pb.h"
#include <QQmlListProperty>
#include "qmlentity.h"

class QmlLayer : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString id READ id WRITE setId NOTIFY idChanged)
    Q_PROPERTY(QQmlListProperty<QmlEntity> entities READ entities)

public:
    QmlLayer(upns::Layer* obj);
    ~QmlLayer();

    QString id() const
    {
        return QString::number(m_layer->id());
    }

    QQmlListProperty<QmlEntity> entities()
    {
        return QQmlListProperty<QmlEntity>(this, NULL, entities_count, entities_at);
    }

    Q_INVOKABLE QmlEntity *addEntity();

public Q_SLOTS:


    void setId(QString id)
    {
        ::google::protobuf::int64 ident = id.toULongLong();
        if (m_layer->id() == ident)
            return;

        m_layer->set_id(ident);
        Q_EMIT idChanged(id);
    }

Q_SIGNALS:
    void idChanged(QString id);

private:
    upns::Layer *m_layer;
    QList<QmlEntity*> m_entities;
    //static void entities_append(QQmlListProperty<QmlEntity> *property, QmlEntity *value);
    static int entities_count(QQmlListProperty<QmlEntity> *property);
    static QmlEntity* entities_at(QQmlListProperty<QmlEntity> *property, int index);
    //static void entities_clear(QQmlListProperty<QmlEntity> *property);
};

#endif
