#ifndef RENDERTHREAD_H
#define RENDERTHREAD_H

#include <QOpenGLFramebufferObject>
#include <QOffscreenSurface>
#include <QMatrix4x4>
#include "bindings/qmlentitydata.h"

#include <OVR_CAPI_GL.h>
class MapsRenderer;

class TextureBuffer;
class DepthBuffer;
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
    Q_PROPERTY(bool vrmode READ vrmode WRITE setVrmode NOTIFY vrmodeChanged)

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

    bool vrmode() const
    {
        return m_vrmode;
    }

public Q_SLOTS:
    void reload();
    void renderNextVR();
    void renderNextNonVR();
    void renderNext();
    void shutDown();

    void setWidth(qreal width);
    void setHeight(qreal height);
    void setMatrix(QMatrix4x4 matrix);

    void setEntitydata(QmlEntitydata *entitydata);

    void setVrmode(bool vrmode)
    {
        if (m_vrmode == vrmode)
            return;

        m_vrmode = vrmode;
        Q_EMIT vrmodeChanged(vrmode);
    }

Q_SIGNALS:
    void textureReady(int id, const QSize &size);

    void widthChanged(qreal width);
    void heightChanged(qreal height);
    void matrixChanged(QMatrix4x4 matrix);

    void entitydataChanged(QmlEntitydata *entitydata);

    void vrmodeChanged(bool vrmode);

private:
    QOpenGLFramebufferObject *m_renderFbo;
    QOpenGLFramebufferObject *m_displayFbo;

    void initVR();
    bool m_vrInitialized;
    QSize m_size;
    QMatrix4x4 m_matrix;
    QmlEntitydata *m_entitydata;
    MapsRenderer *m_mapsRenderer;
    ovrEyeRenderDesc m_eyeRenderDesc[2];
    ovrHmd m_HMD;
    TextureBuffer *m_eyeRenderTexture[2];
    DepthBuffer   *m_eyeDepthBuffer[2];
    ovrGLTexture  *m_mirrorTexture;
    GLuint         m_mirrorFBO;
    ovrHmdDesc m_hmdDesc;
    bool m_vrmode;
};

#endif // MapsRenderer_H
