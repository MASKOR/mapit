#ifndef MapsRenderer_H
#define MapsRenderer_H

#include <QQuickItem>
#include "qmlmapmanager.h"

class RenderThread;

class MapsRenderViewport : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(QString mapId READ mapId WRITE setMapId NOTIFY mapIdChanged)
    Q_PROPERTY(QmlMapManager *mapManager READ mapManager WRITE setMapManager NOTIFY mapManagerChanged)

public:
    MapsRenderViewport();

    static QList<QThread *> threads;

    QString mapId() const
    {
        return m_mapId;
    }

    QmlMapManager * mapManager() const
    {
        return m_mapManager;
    }

public Q_SLOTS:
    void ready();

    void setMapId(QString mapId)
    {
        if (m_mapId == mapId)
            return;

        m_mapId = mapId;
        Q_EMIT mapIdChanged(mapId);
    }

    void setMapManager(QmlMapManager * mapManager)
    {
        if (m_mapManager == mapManager)
            return;

        m_mapManager = mapManager;
        Q_EMIT mapManagerChanged(mapManager);
    }

    void reload();

Q_SIGNALS:
    void mapIdChanged(QString mapId);

    void mapManagerChanged(QmlMapManager * mapManager);

    void needsReload();
protected:
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);

private:
    RenderThread *m_renderThread;
    QString m_mapId;
    QmlMapManager *m_mapManager;
};

#endif // MapsRenderer_H
