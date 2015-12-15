#ifndef MapsRenderer_H
#define MapsRenderer_H

#include <QQuickItem>
#include <QMatrix4x4>
#include "../bindings/qmlmapmanager.h"

class RenderThread;

class MapsRenderViewport : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(QString mapId READ mapId WRITE setMapId NOTIFY mapIdChanged)
    Q_PROPERTY(QmlMapManager *mapManager READ mapManager WRITE setMapManager NOTIFY mapManagerChanged)
    Q_PROPERTY(QMatrix4x4 matrix READ matrix WRITE setMatrix NOTIFY matrixChanged)

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

    QMatrix4x4 matrix() const
    {
        return m_matrix;
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

    void setMatrix(QMatrix4x4 matrix)
    {
        if (m_matrix == matrix)
            return;

        m_matrix = matrix;
        Q_EMIT matrixChanged(matrix);
    }

Q_SIGNALS:
    void mapIdChanged(QString mapId);

    void mapManagerChanged(QmlMapManager * mapManager);

    void needsReload();
    void matrixChanged(QMatrix4x4 matrix);

protected:
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);

private:
    RenderThread *m_renderThread;
    QString m_mapId;
    QmlMapManager *m_mapManager;
    QMatrix4x4 m_matrix;
};

#endif // MapsRenderer_H
