#ifndef RENDERTHREAD_H
#define RENDERTHREAD_H

#include <QOpenGLFramebufferObject>
#include <QOffscreenSurface>
#include <QMatrix4x4>
#include "bindings/qmlentitydata.h"

class MapsRenderer;

/*
 * The render thread shares a context with the scene graph and will
 * render into two separate FBOs, one to use for display and one
 * to use for rendering
 */
class RenderThread : public QThread
{
    Q_OBJECT
    Q_PROPERTY(QmlEntitydata *entitydata READ entitydata WRITE setEntitydata NOTIFY entitydataChanged)
    Q_PROPERTY(qreal width READ width WRITE setWidth NOTIFY widthChanged)
    Q_PROPERTY(qreal height READ height WRITE setHeight NOTIFY heightChanged)
    Q_PROPERTY(QMatrix4x4 matrix READ matrix WRITE setMatrix NOTIFY matrixChanged)

public:
    RenderThread(const QSize &size);
    virtual ~RenderThread();

    const MapsRenderer* mapsRenderer();
    QOffscreenSurface *surface;
    QOpenGLContext *context;

    qreal width() const;
    qreal height() const;
    QMatrix4x4 matrix() const;

    QmlEntitydata *entitydata() const;

public Q_SLOTS:
    void reload();
    void renderNext();
    void shutDown();

    void setWidth(qreal width);
    void setHeight(qreal height);
    void setMatrix(QMatrix4x4 matrix);

    void setEntitydata(QmlEntitydata *entitydata);

Q_SIGNALS:
    void textureReady(int id, const QSize &size);

    void widthChanged(qreal width);
    void heightChanged(qreal height);
    void matrixChanged(QMatrix4x4 matrix);

    void entitydataChanged(QmlEntitydata *entitydata);

private:
    QOpenGLFramebufferObject *m_renderFbo;
    QOpenGLFramebufferObject *m_displayFbo;

    QSize m_size;
    QMatrix4x4 m_matrix;
    QmlEntitydata *m_entitydata;
    MapsRenderer *m_mapsRenderer;
};

#endif // MapsRenderer_H
