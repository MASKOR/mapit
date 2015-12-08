#ifndef QMLMAPMANAGERCONFIG_H
#define QMLMAPMANAGERCONFIG_H

#include <QObject>

class QmlMapManagerConfig : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString mapSource READ mapSource WRITE setMapSource NOTIFY mapSourceChanged)

public:
    QString mapSource() const
    {
        return m_mapSource;
    }

public Q_SLOTS:
    void setMapSource(QString mapSource)
    {
        if (m_mapSource == mapSource)
            return;

        m_mapSource = mapSource;
        Q_EMIT mapSourceChanged(mapSource);
    }

Q_SIGNALS:
    void mapSourceChanged(QString mapSource);

private:
    QString m_mapSource;

};
#endif
