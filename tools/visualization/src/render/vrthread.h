#ifndef VRTHREAD_H
#define VRTHREAD_H

#include <QOpenGLFramebufferObject>
#include <QOffscreenSurface>
#include <QMatrix4x4>
#include <QThread>
#include "upns/ui/bindings/qmlentitydata.h"
#include <QOpenGLDebugMessage>

#ifdef VRMODE
#include <OVR_CAPI_GL.h>
#endif
class MapsRenderer;

class TextureBuffer;
class DepthBuffer;
/*
 * TODO: Get new independent OpenGL Context and use shared texture
 */
class VRThread : public QThread
{
    Q_OBJECT

public:
    VRThread(const QSize &size);
    virtual ~VRThread();
};

#endif // MapsRenderer_H
