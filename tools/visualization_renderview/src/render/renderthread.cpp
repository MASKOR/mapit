
#include "renderthread.h"

#include <QMutex>
#include <QThread>

#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QGuiApplication>
#include <QOffscreenSurface>
#include <QOpenGLFunctions_4_1_Core>

#include <QQuickWindow>
#include <QSGSimpleTextureNode>

#ifdef VRMODE
#include <Kernel/OVR_System.h>
#include <OVR_CAPI_GL.h>
#include <Extras/OVR_Math.h>
#endif

#include "mapsrenderer.h"
#include "bindings/qmlmapsrenderviewport.h"

#ifdef VRMODE
////Copy Paste OVR
//--------------------------------------------------------------------------
struct DepthBuffer
{
    GLuint        texId;

    DepthBuffer(OVR::Sizei size, int sampleCount)
    {
        QOpenGLFunctions_4_1_Core funcs;
        funcs.initializeOpenGLFunctions();
        OVR_ASSERT(sampleCount <= 1); // The code doesn't currently handle MSAA textures.

        funcs.glGenTextures(1, &texId);
        funcs.glBindTexture(GL_TEXTURE_2D, texId);
        funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        GLenum internalFormat = GL_DEPTH_COMPONENT24;
        GLenum type = GL_UNSIGNED_INT;
        if (true)//GLE_ARB_depth_buffer_float)
        {
            internalFormat = GL_DEPTH_COMPONENT32F;
            type = GL_FLOAT;
        }

        funcs.glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, size.w, size.h, 0, GL_DEPTH_COMPONENT, type, NULL);
    }
    ~DepthBuffer()
    {
        if (texId)
        {
            QOpenGLFunctions_4_1_Core funcs;
            funcs.initializeOpenGLFunctions();
            funcs.glDeleteTextures(1, &texId);
            texId = 0;
        }
    }
};
struct TextureBuffer
{
    ovrHmd              hmd;
    ovrSwapTextureSet*  TextureSet;
    GLuint              texId;
    GLuint              fboId;
    OVR::Sizei          texSize;

    TextureBuffer(ovrHmd hmd, bool rendertarget, bool displayableOnHmd, OVR::Sizei size, int mipLevels, unsigned char * data, int sampleCount) :
        hmd(hmd),
        TextureSet(nullptr),
        texId(0),
        fboId(0),
        texSize(0, 0)
    {
        OVR_ASSERT(sampleCount <= 1); // The code doesn't currently handle MSAA textures.

        texSize = size;
        QOpenGLFunctions_4_1_Core funcs;
        funcs.initializeOpenGLFunctions();

        if (displayableOnHmd)
        {
            // This texture isn't necessarily going to be a rendertarget, but it usually is.
            OVR_ASSERT(hmd); // No HMD? A little odd.
            OVR_ASSERT(sampleCount == 1); // ovr_CreateSwapTextureSetD3D11 doesn't support MSAA.

            ovrResult result = ovr_CreateSwapTextureSetGL(hmd, GL_SRGB8_ALPHA8, size.w, size.h, &TextureSet);

            if(OVR_SUCCESS(result))
            {
                for (int i = 0; i < TextureSet->TextureCount; ++i)
                {
                    ovrGLTexture* tex = (ovrGLTexture*)&TextureSet->Textures[i];
                    funcs.glBindTexture(GL_TEXTURE_2D, tex->OGL.TexId);

                    if (rendertarget)
                    {
                        funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                        funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                        funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                        funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                    }
                    else
                    {
                        funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                        funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                        funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
                        funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
                    }
                }
            }
        }
        else
        {
            funcs.glGenTextures(1, &texId);
            funcs.glBindTexture(GL_TEXTURE_2D, texId);

            if (rendertarget)
            {
                funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            }
            else
            {
                funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
                funcs.glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            }

            funcs.glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8_ALPHA8, texSize.w, texSize.h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        }

        if (mipLevels > 1)
        {
            funcs.glGenerateMipmap(GL_TEXTURE_2D);
        }

        funcs.glGenFramebuffers(1, &fboId);
    }

    ~TextureBuffer()
    {
        QOpenGLFunctions_4_1_Core funcs;
        funcs.initializeOpenGLFunctions();
        if (TextureSet)
        {
            ovr_DestroySwapTextureSet(hmd, TextureSet);
            TextureSet = nullptr;
        }
        if (texId)
        {
            funcs.glDeleteTextures(1, &texId);
            texId = 0;
        }
        if (fboId)
        {
            funcs.glDeleteFramebuffers(1, &fboId);
            fboId = 0;
        }
    }

    OVR::Sizei GetSize() const
    {
        return texSize;
    }

    void SetAndClearRenderSurface(DepthBuffer* dbuffer)
    {
        QOpenGLFunctions_4_1_Core funcs;
        funcs.initializeOpenGLFunctions();
        auto tex = reinterpret_cast<ovrGLTexture*>(&TextureSet->Textures[TextureSet->CurrentIndex]);

        funcs.glBindFramebuffer(GL_FRAMEBUFFER, fboId);
        funcs.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex->OGL.TexId, 0);
        funcs.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, dbuffer->texId, 0);

        funcs.glViewport(0, 0, texSize.w, texSize.h);
        funcs.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        funcs.glEnable(GL_FRAMEBUFFER_SRGB);
    }

    void UnsetRenderSurface()
    {
        QOpenGLFunctions_4_1_Core funcs;
        funcs.initializeOpenGLFunctions();
        funcs.glBindFramebuffer(GL_FRAMEBUFFER, fboId);
        funcs.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
        funcs.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
    }
};
////
#endif

RenderThread::RenderThread(const QSize &size)
    : surface(0)
    , context(0)
    , m_renderFbo(0)
    , m_displayFbo(0)
    , m_vrInitialized(false)
    , m_size(size)
    , m_entitydata(NULL)
    #ifdef VRMODE
    , m_mirrorTexture(nullptr)
    , m_mirrorFBO( 0 )
    #endif
    , m_vrmode( false ),
      m_running( false )
{
#ifdef VRMODE
    m_eyeRenderTexture[0] = nullptr;
    m_eyeRenderTexture[1] = nullptr;
    m_eyeDepthBuffer[0] = nullptr;
    m_eyeDepthBuffer[1] = nullptr;
#endif
    QmlMapsRenderViewport::threads << this;
    m_mapsRenderer = new MapsRenderer();
}

RenderThread::~RenderThread()
{
    delete m_mapsRenderer;
}

qreal RenderThread::width() const
{
    return m_size.width();
}

qreal RenderThread::height() const
{
    return m_size.height();
}

QMatrix4x4 RenderThread::matrix() const
{
    return m_matrix;
}

void RenderThread::renderNext()
{
#ifdef VRMODE
    if(m_vrmode)
    {
        renderNextVR();
    }
    else
    {
#endif
        renderNextNonVR();
#ifdef VRMODE
    }
#endif
}

void RenderThread::renderNextNonVR()
{
    context->makeCurrent(surface);
    if(m_renderFbo && m_renderFbo->size() != m_size)
    {
        delete m_renderFbo;
        m_renderFbo = NULL;
        delete m_displayFbo;
        m_displayFbo = NULL;
    }
    if (!m_renderFbo)
    {
        // Initialize the buffers and renderer
        QOpenGLFramebufferObjectFormat format;
        format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
        m_renderFbo = new QOpenGLFramebufferObject(m_size, format);
        m_displayFbo = new QOpenGLFramebufferObject(m_size, format);
    }
    if( !m_mapsRenderer->isInitialized() && entitydata() != NULL && entitydata()->getEntityData() != NULL)
    {
        m_mapsRenderer->setEntityData(entitydata()->getEntityData());
        m_mapsRenderer->initialize();
        setRunning(true);
    }
    m_renderFbo->bind();
    context->functions()->glViewport(0, 0, m_size.width(), m_size.height());
    QMatrix4x4 mat;
    m_mapsRenderer->render(mat, mat );

    // We need to flush the contents to the FBO before posting
    // the texture to the other thread, otherwise, we might
    // get unexpected results.
    context->functions()->glFlush();

    m_renderFbo->bindDefault();
    qSwap(m_renderFbo, m_displayFbo);

    Q_EMIT textureReady(m_displayFbo->texture(), m_size);
}

#ifdef VRMODE
void RenderThread::renderNextVR()
{
    context->makeCurrent(surface);

    if(!m_vrInitialized)
    {
        initVR();
    }
    if(m_renderFbo && m_renderFbo->size() != m_size)
    {
        delete m_renderFbo;
        m_renderFbo = NULL;
        delete m_displayFbo;
        m_displayFbo = NULL;
    }
    if (!m_renderFbo)
    {
        // Initialize the buffers and renderer
        QOpenGLFramebufferObjectFormat format;
        format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
        m_renderFbo = new QOpenGLFramebufferObject(m_size, format);
        m_displayFbo = new QOpenGLFramebufferObject(m_size, format);
    }
    if( !m_mapsRenderer->isInitialized() && entitydata() != NULL && entitydata()->getEntityData() != NULL)
    {
        m_mapsRenderer->setEntityData(entitydata()->getEntityData());
        m_mapsRenderer->initialize();
        setRunning(true);
    }





    static float Yaw(3.141592f);
//    if (Platform.Key[VK_LEFT])  Yaw += 0.02f;
//    if (Platform.Key[VK_RIGHT]) Yaw -= 0.02f;

    // Keyboard inputs to adjust player position
    static OVR::Vector3f Pos2(0.0f,1.6f,-5.0f);
//    if (Platform.Key['W']||Platform.Key[VK_UP])     Pos2+=Matrix4f::RotationY(Yaw).Transform(Vector3f(0,0,-0.05f));
//    if (Platform.Key['S']||Platform.Key[VK_DOWN])   Pos2+=Matrix4f::RotationY(Yaw).Transform(Vector3f(0,0,+0.05f));
//    if (Platform.Key['D'])                          Pos2+=Matrix4f::RotationY(Yaw).Transform(Vector3f(+0.05f,0,0));
//    if (Platform.Key['A'])                          Pos2+=Matrix4f::RotationY(Yaw).Transform(Vector3f(-0.05f,0,0));
//    Pos2.y = ovr_GetFloat(HMD, OVR_KEY_EYE_HEIGHT, Pos2.y);

    // Get eye poses, feeding in correct IPD offset
    ovrVector3f               ViewOffset[2] = { m_eyeRenderDesc[0].HmdToEyeViewOffset,
                                                m_eyeRenderDesc[1].HmdToEyeViewOffset };
    ovrPosef                  EyeRenderPose[2];

    double           ftiming = ovr_GetPredictedDisplayTime(m_HMD, 0);
    // Keeping sensorSampleTime as close to ovr_GetTrackingState as possible - fed into the layer
    double           sensorSampleTime = ovr_GetTimeInSeconds();
    ovrTrackingState hmdState = ovr_GetTrackingState(m_HMD, ftiming, ovrTrue);
    ovr_CalcEyePoses(hmdState.HeadPose.ThePose, ViewOffset, EyeRenderPose);

    if (true)
    {
        for (int eye = 0; eye < 2; ++eye)
        {
            // Increment to use next texture, just before writing
            m_eyeRenderTexture[eye]->TextureSet->CurrentIndex = (m_eyeRenderTexture[eye]->TextureSet->CurrentIndex + 1) % m_eyeRenderTexture[eye]->TextureSet->TextureCount;

            // Switch to eye render target
            m_eyeRenderTexture[eye]->SetAndClearRenderSurface(m_eyeDepthBuffer[eye]);

            // Get view and projection matrices
            OVR::Matrix4f rollPitchYaw = OVR::Matrix4f::RotationY(Yaw);
            OVR::Quatf orientation(EyeRenderPose[eye].Orientation);
            OVR::Matrix4f finalRollPitchYaw = rollPitchYaw * OVR::Matrix4f(orientation);
            OVR::Vector3f finalUp = finalRollPitchYaw.Transform(OVR::Vector3f(0, 1, 0));
            OVR::Vector3f finalForward = finalRollPitchYaw.Transform(OVR::Vector3f(0, 0, -1));
            OVR::Vector3f shiftedEyePos = Pos2 + rollPitchYaw.Transform(EyeRenderPose[eye].Position);

            OVR::Matrix4f view = OVR::Matrix4f::LookAtRH(shiftedEyePos, shiftedEyePos + finalForward, finalUp);
            OVR::Matrix4f proj = ovrMatrix4f_Projection(m_hmdDesc.DefaultEyeFov[eye], 0.2f, 1000.0f, ovrProjection_RightHanded);
            QMatrix4x4 qview(&view.M[0][0]);
            QMatrix4x4 qproj(&proj.M[0][0]);

            QMatrix4x4 qorientation(&finalRollPitchYaw.M[0][0]);//QQuaternion(orientation.w, orientation.x, orientation.y, orientation.z));
            QVector3D qdir(finalForward.x, finalForward.y, finalForward.z);
            setHeadMatrix(qview);
            setHeadDirection(qdir);
            setHeadOrientation(qorientation);
            // Render world
            //roomScene->Render(view, proj);
            //context->functions()->glViewport(0, 0, m_size.width(), m_size.height());
            m_mapsRenderer->render(qview, qproj);

            // Avoids an error when calling SetAndClearRenderSurface during next iteration.
            // Without this, during the next while loop iteration SetAndClearRenderSurface
            // would bind a framebuffer with an invalid COLOR_ATTACHMENT0 because the texture ID
            // associated with COLOR_ATTACHMENT0 had been unlocked by calling wglDXUnlockObjectsNV.
            m_eyeRenderTexture[eye]->UnsetRenderSurface();
        }
    }

    // Do distortion rendering, Present and flush/sync

    // Set up positional data.
    ovrViewScaleDesc viewScaleDesc;
    viewScaleDesc.HmdSpaceToWorldScaleInMeters = 1.0f;
    viewScaleDesc.HmdToEyeViewOffset[0] = ViewOffset[0];
    viewScaleDesc.HmdToEyeViewOffset[1] = ViewOffset[1];

    ovrLayerEyeFov ld;
    ld.Header.Type  = ovrLayerType_EyeFov;
    ld.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft;   // Because OpenGL.

    for (int eye = 0; eye < 2; ++eye)
    {
        ld.ColorTexture[eye] = m_eyeRenderTexture[eye]->TextureSet;
        ld.Viewport[eye]     = OVR::Recti(m_eyeRenderTexture[eye]->GetSize());
        ld.Fov[eye]          = m_hmdDesc.DefaultEyeFov[eye];
        ld.RenderPose[eye]   = EyeRenderPose[eye];
        ld.SensorSampleTime  = sensorSampleTime;
    }

    ovrLayerHeader* layers = &ld.Header;
    ovrResult result = ovr_SubmitFrame(m_HMD, 0, &viewScaleDesc, &layers, 1);
    // exit the rendering loop if submit returns an error, will retry on ovrError_DisplayLost
    if (!OVR_SUCCESS(result)) return;
//        goto Done;

    //isVisible = (result == ovrSuccess);

    QOpenGLFunctions_4_1_Core funcs;
    funcs.initializeOpenGLFunctions();
    if(m_mirrorEnabled)
    {
        // Blit mirror texture to back buffer
        GLint w;
        GLint h;
        if(m_mirrorDistorsion)
        {
            w = m_eyeRenderTexture[1]->texSize.w;
            h = m_eyeRenderTexture[1]->texSize.h;
        }
        else if(m_mirrorRightEye)
        {
            w = m_eyeRenderTexture[0]->texSize.w;
            h = m_eyeRenderTexture[0]->texSize.h;
        }
        else
        {
            w = m_mirrorTexture->OGL.Header.TextureSize.w;
            h = m_mirrorTexture->OGL.Header.TextureSize.h;
        }
        funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, m_mirrorFBO);


        funcs.glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_renderFbo->handle());
        funcs.glBlitFramebuffer(0, 0, w, h,
                          0, 0, w, h,
                          GL_COLOR_BUFFER_BIT, GL_NEAREST);
        funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    }
//    SwapBuffers(Platform.hDC);


    m_renderFbo->bind();
    //context->functions()->glViewport(0, 0, m_size.width(), m_size.height());

    //m_mapsRenderer->render();

    // We need to flush the contents to the FBO before posting
    // the texture to the other thread, otherwise, we might
    // get unexpected results.
    context->functions()->glFlush();

    m_renderFbo->bindDefault();
    qSwap(m_renderFbo, m_displayFbo);

    Q_EMIT textureReady(m_displayFbo->texture(), m_size);

}
#endif

void RenderThread::shutDown()
{
    context->makeCurrent(surface);
    delete m_renderFbo;
    delete m_displayFbo;
    context->doneCurrent();
    delete context;

    // schedule this to be deleted only after we're done cleaning up
    surface->deleteLater();

    // Stop event processing, move the thread to GUI and make sure it is deleted.
    QCoreApplication* app = QGuiApplication::instance();
    QThread *guiThread = app->thread();
    moveToThread(guiThread);
    exit();
}

void RenderThread::setWidth(qreal width)
{
    if (m_size.width() == width)
        return;

    m_size.setWidth(width);
    Q_EMIT widthChanged(width);
}

void RenderThread::setHeight(qreal height)
{
    if (m_size.height() == height)
        return;

    m_size.setHeight(height);
    Q_EMIT heightChanged(height);
}

void RenderThread::setMatrix(QMatrix4x4 matrix)
{
    if (m_matrix == matrix)
        return;

    m_matrix = matrix;
    if(m_mapsRenderer)
        m_mapsRenderer->setMatrix( matrix );
    Q_EMIT matrixChanged(matrix);
}

void RenderThread::setEntitydata(QmlEntitydata *entitydata)
{
    if (m_entitydata == entitydata)
        return;

    m_entitydata = entitydata;
    m_mapsRenderer->setEntityData(m_entitydata->getEntityData());
    Q_EMIT entitydataChanged(entitydata);
}

void RenderThread::setMirrorEnabled(bool mirrorEnabled)
{
    if (m_mirrorEnabled == mirrorEnabled)
        return;

    m_mirrorEnabled = mirrorEnabled;
    Q_EMIT mirrorEnabledChanged(mirrorEnabled);
}

void RenderThread::setMirrorDistorsion(bool mirrorDistorsion)
{
    if (m_mirrorDistorsion == mirrorDistorsion)
        return;

    m_mirrorDistorsion = mirrorDistorsion;
    if(m_mirrorDistorsion)
    {
        QOpenGLFunctions_4_1_Core funcs;
        funcs.initializeOpenGLFunctions();
        funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, m_mirrorFBO);
        funcs.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_mirrorTexture->OGL.TexId, 0);
        funcs.glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
        funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    }
    Q_EMIT mirrorDistorsionChanged(mirrorDistorsion);
}

void RenderThread::setMirrorRightEye(bool mirrorRightEye)
{
    if (m_mirrorRightEye == mirrorRightEye)
        return;

    m_mirrorRightEye = mirrorRightEye;
    if(!m_mirrorDistorsion)
    {
//        if(m_mirrorRightEye)
//        {
//            QOpenGLFunctions_4_1_Core funcs;
//            funcs.initializeOpenGLFunctions();
//            funcs.glDeleteFramebuffers(1, &m_mirrorFBO);
//            funcs.glGenFramebuffers(1, &m_mirrorFBO);
//            funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, m_mirrorFBO);
//            funcs.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_eyeRenderTexture[1]->texId, 0);
//            //funcs.glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
//            funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
//        }
//        else
//        {
//            QOpenGLFunctions_4_1_Core funcs;
//            funcs.initializeOpenGLFunctions();
//            funcs.glDeleteFramebuffers(1, &m_mirrorFBO);
//            funcs.glGenFramebuffers(1, &m_mirrorFBO);
//            funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, m_mirrorFBO);
//            funcs.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_eyeRenderTexture[0]->texId, 0);
//            //funcs.glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
//            funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
//        }
    }
    Q_EMIT mirrorRightEyeChanged(mirrorRightEye);
}

void RenderThread::setRunning(bool running)
{
    if (m_running == running)
        return;

    m_running = running;
    Q_EMIT runningChanged(running);
}

void RenderThread::setHeadOrientation(QMatrix4x4 headOrientation)
{
    //    if (m_headOrientation == headOrientation)
    //        return;

    //    m_headOrientation = headOrientation;
    Q_EMIT headOrientationChanged(headOrientation);
}

void RenderThread::setHeadMatrix(QMatrix4x4 headMatrix)
{
//    if (m_headMatrix == headMatrix)
//        return;

//    m_headMatrix = headMatrix;
    Q_EMIT headMatrixChanged(headMatrix);
}

void RenderThread::setHeadDirection(QVector3D headDirection)
{
//    if (m_headDirection == headDirection)
//        return;

//    m_headDirection = headDirection;
    Q_EMIT headDirectionChanged(headDirection);
}

#ifdef VRMODE
void RenderThread::initVR()
{
    ovrResult result = ovr_Initialize(nullptr);
    if (!OVR_SUCCESS(result))
        return;
    ovrGraphicsLuid luid;
    result = ovr_Create(&m_HMD, &luid);
    if (!OVR_SUCCESS(result))
        return;

    m_hmdDesc = ovr_GetHmdDesc(m_HMD);

    //ovrSizei windowSize = { m_hmdDesc.Resolution.w / 2, m_hmdDesc.Resolution.h / 2 };
    ovrSizei windowSize = { m_size.width(), m_size.height() };

    // Make eye render buffers
    for (int eye = 0; eye < 2; ++eye)
    {
        ovrSizei idealTextureSize = ovr_GetFovTextureSize(m_HMD, ovrEyeType(eye), m_hmdDesc.DefaultEyeFov[eye], 1);
        m_eyeRenderTexture[eye] = new TextureBuffer(m_HMD, true, true, idealTextureSize, 1, NULL, 1);
        m_eyeDepthBuffer[eye]   = new DepthBuffer(m_eyeRenderTexture[eye]->GetSize(), 0);

        if (!m_eyeRenderTexture[eye]->TextureSet)
        {
            if (true) return;// goto Done;
            //VALIDATE(false, "Failed to create texture.");
        }
    }

    // Create mirror texture and an FBO used to copy mirror texture to back buffer
    result = ovr_CreateMirrorTextureGL(m_HMD, GL_SRGB8_ALPHA8, windowSize.w, windowSize.h, reinterpret_cast<ovrTexture**>(&m_mirrorTexture));
    if (!OVR_SUCCESS(result))
    {
        if (true) return;// goto Done;
        //VALIDATE(false, "Failed to create mirror texture.");
    }

    QOpenGLFunctions_4_1_Core funcs;
    funcs.initializeOpenGLFunctions();
    // Configure the mirror read buffer
    funcs.glGenFramebuffers(1, &m_mirrorFBO);
    funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, m_mirrorFBO);
    funcs.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_mirrorTexture->OGL.TexId, 0);
    funcs.glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
    funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

    m_eyeRenderDesc[0] = ovr_GetRenderDesc(m_HMD, ovrEye_Left, m_hmdDesc.DefaultEyeFov[0]);
    m_eyeRenderDesc[1] = ovr_GetRenderDesc(m_HMD, ovrEye_Right, m_hmdDesc.DefaultEyeFov[1]);

    // Turn off vsync to let the compositor do its magic
//    wglSwapIntervalEXT(0);


    m_vrInitialized = true;
//    Done:
    return;
}
#endif

QmlEntitydata *RenderThread::entitydata() const
{
    return m_entitydata;
}

void RenderThread::reload()
{

}
