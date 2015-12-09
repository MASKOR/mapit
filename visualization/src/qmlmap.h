#ifndef QMLMAP_H
#define QMLMAP_H

#include <QtCore>
#include <QJsonObject>
#include "mapmanager/src/mapmanager.h"
#include "upns_interface/services.pb.h"
#include <QQmlListProperty>
#include "qmllayer.h"

class QmlMap : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString id READ id WRITE setId NOTIFY idChanged)
    Q_PROPERTY(QString name READ name WRITE setName NOTIFY nameChanged)
    Q_PROPERTY(QQmlListProperty<QmlLayer> layers READ layers)

public:
    QmlMap(upns::Map* obj);
    ~QmlMap();

    QString id() const
    {
        return QString::number(m_map->id());
    }

    QString name() const
    {
        return QString::fromStdString(m_map->name());
    }

    Q_INVOKABLE QmlLayer *addLayer();

    QQmlListProperty<QmlLayer> layers()
    {
        return QQmlListProperty<QmlLayer>(this, NULL, layers_count, layers_at);
    }

public Q_SLOTS:


    void setId(QString id)
    {
        ::google::protobuf::int64 ident = id.toULongLong();
        if (m_map->id() == ident)
            return;

        m_map->set_id(ident);
        Q_EMIT idChanged(id);
    }

    void setName(QString name)
    {
        ::std::string str = name.toStdString();
        if (m_map->name() == str)
            return;

        m_map->set_name(str);
        Q_EMIT nameChanged(name);
    }

Q_SIGNALS:
    void idChanged(QString id);
    void nameChanged(QString name);

private:
    upns::Map* m_map;
    QList<QmlLayer*> m_layers;

    static int layers_count(QQmlListProperty<QmlLayer> *property);
    static QmlLayer *layers_at(QQmlListProperty<QmlLayer> *property, int index);
};

#endif
