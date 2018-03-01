/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2015 Tobias Neumann	<t.neumann@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RENDERTHREAD_H
#define RENDERTHREAD_H

#include <QOpenGLFramebufferObject>
#include <QOffscreenSurface>
#include <QMatrix4x4>
#include <QThread>
#include "upns/ui/bindings/qmlentitydata.h"
#include <QOpenGLDebugMessage>
#include "upns/ui/bindings/renderdata.h"
#include <boost/timer.hpp>

#ifdef VRMODE
#include <OVR_CAPI_GL.h>
#endif

#define FRAMES_AVERAGE_WINDOW 100
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
    Q_PROPERTY(Renderdata *renderdata READ renderdata)
public:
    RenderThread();
    virtual ~RenderThread();

    const MapsRenderer* mapsRenderer();
    QOffscreenSurface *surface;
    QOpenGLContext *context;

    Renderdata *renderdata()
    {
        return &m_renderdata;
    }

public Q_SLOTS:
    void onMessageLogged( QOpenGLDebugMessage message);
    void reload();
    #ifdef VRMODE
    void renderNextVR();
    void vrThreadMainloop();
    #endif
    void renderNextNonVR();
    void renderNext();
    void shutDown();

Q_SIGNALS:
    void updated();
    void frameFinished();

    void textureReady(int id, const QSize &size);

private Q_SLOTS:
//    void setHeadOrientation(QMatrix4x4 headOrientation);
//    void setHeadMatrix(QMatrix4x4 headMatrix);
//    void setHeadDirection(QVector3D headDirection);
//    void setRunning(bool running);
    void initialize();
private:

    void resetMirror();
    QOpenGLFramebufferObject *m_renderFbo;
    QOpenGLFramebufferObject *m_displayFbo;

    bool m_vrInitialized;
    MapsRenderer *m_mapsRenderer;
#ifdef VRMODE
void initVR();
    ovrEyeRenderDesc m_eyeRenderDesc[2];
    ovrHmd m_HMD;
    TextureBuffer *m_eyeRenderTexture[2];
    DepthBuffer   *m_eyeDepthBuffer[2];
    ovrGLTexture  *m_mirrorTexture;
    GLuint         m_mirrorFBO;
    ovrHmdDesc m_hmdDesc;
    std::shared_ptr<QMetaObject::Connection> m_vrMainLoopConnection;
#endif
    QOpenGLDebugLogger m_logger;
    Renderdata m_renderdata;
    boost::timer m_timer;
    int m_framecount;
    int m_frameIndex;
    double m_frametimes[FRAMES_AVERAGE_WINDOW][3];
};

#endif // MapsRenderer_H
