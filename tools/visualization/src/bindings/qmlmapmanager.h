#ifndef QMLMAPMANAGER_H
#define QMLMAPMANAGER_H

#include <QtCore>
#include <QJsonObject>
#include "upns_globals.h"
#include "versioning/checkout.h"
#include "services.pb.h"
#include "qmlmap.h"
#include "yaml-cpp/yaml.h"

class QmlMapManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QJsonObject config READ config WRITE setConfig NOTIFY configChanged)
    Q_CLASSINFO("DefaultProperty", "config")

public:
    QmlMapManager();
    ~QmlMapManager();

    Q_INVOKABLE QList<QString> listMaps();
    Q_INVOKABLE QmlMap *getMap(const upns::ObjectId &mapId);

    Q_INVOKABLE bool canRead();
    Q_INVOKABLE bool canWrite();

    Q_INVOKABLE QJsonObject doOperation(const QJsonObject &desc);
    QJsonObject config() const
    {
        return m_config;
    }

    upns::Checkout *getMapManager();

public Q_SLOTS:
    void setConfig(QJsonObject config)
    {
        if (m_config == config)
            return;

        m_config = config;
        Q_EMIT configChanged(config);
    }

Q_SIGNALS:
    void configChanged(QJsonObject config);

private:
    QJsonObject m_config;
    upns::Checkout *m_mapManager;

    void initialize();
    void yamlFromJsonObject(YAML::Node &yaml, const QJsonObject &json) const;
    void operationDescriptionFromJsonObject(upns::OperationDescription &opDesc, const QJsonObject &json) const;
    QJsonObject operationDescriptionToJson(const upns::OperationDescription &desc) const;
    QMap<upns::MapIdentifier, QmlMap*> m_maps;
};

#endif
