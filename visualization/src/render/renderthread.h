#ifndef RENDERTHREAD_H
#define RENDERTHREAD_H

#include <QOpenGLFramebufferObject>
#include <QOffscreenSurface>
#include <QMatrix4x4>
#include "../bindings/qmlmapmanager.h"

class MapsRenderer;

/*
 * The render thread shares a context with the scene graph and will
 * render into two separate FBOs, one to use for display and one
 * to use for rendering
 */
class RenderThread : public QThread
{
    Q_OBJECT
    Q_PROPERTY(QString mapId READ mapId WRITE setMapId NOTIFY mapIdChanged)
    Q_PROPERTY(QString layerId READ layerId WRITE setLayerId NOTIFY layerIdChanged)
    Q_PROPERTY(upns::MapManager *mapManager READ mapManager WRITE setMapManager NOTIFY mapManagerChanged)
    Q_PROPERTY(qreal width READ width WRITE setWidth NOTIFY widthChanged)
    Q_PROPERTY(qreal height READ height WRITE setHeight NOTIFY heightChanged)
    Q_PROPERTY(QMatrix4x4 matrix READ matrix WRITE setMatrix NOTIFY matrixChanged)

public:
    RenderThread(const QSize &size);
    virtual ~RenderThread();

    const MapsRenderer* mapsRenderer();
    QOffscreenSurface *surface;
    QOpenGLContext *context;

    upns::MapManager * mapManager() const;

    QString mapId() const;
    QString layerId() const;
    qreal width() const;
    qreal height() const;
    QMatrix4x4 matrix() const;

public Q_SLOTS:
    void reloadMap();
    void renderNext();
    void shutDown();

    void setMapManager(upns::MapManager * mapManager);
    void setMapId(QString mapId);
    void setLayerId(QString layerId);
    void setWidth(qreal width);
    void setHeight(qreal height);
    void setMatrix(QMatrix4x4 matrix);

Q_SIGNALS:
    void textureReady(int id, const QSize &size);

    void mapManagerChanged(upns::MapManager * mapManager);
    void mapIdChanged(QString mapId);
    void widthChanged(qreal width);
    void heightChanged(qreal height);
    void matrixChanged(QMatrix4x4 matrix);
    void layerIdChanged(QString layerId);

private:
    QOpenGLFramebufferObject *m_renderFbo;
    QOpenGLFramebufferObject *m_displayFbo;

    MapsRenderer *m_mapsRenderer;
    QSize m_size;
    upns::MapManager *m_mapManager;
    QString m_mapId;
    QString m_layerId;
    QMatrix4x4 m_matrix;
};

#endif // MapsRenderer_H
