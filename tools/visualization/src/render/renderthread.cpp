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


#include "renderthread.h"

#include <QMutex>
#include <QThread>

#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QGuiApplication>
#include <QOffscreenSurface>
#include <QOpenGLFunctions_3_0>

#include <QQuickWindow>
#include <QSGSimpleTextureNode>

#ifdef VRMODE
#include <Kernel/OVR_System.h>
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>
#include <Extras/OVR_Math.h>
#endif

#include <numeric>
#include "mapsrenderer.h"
#include "mapit/ui/bindings/qmlmapsrenderviewport.h"

#ifdef VRMODE
//GLfloat viewports[4*4] = {
//0.0f, 0.0f, w_2f, h_2f,
//w_2f, 0.0f, w_2f, h_2f,
//0.0f, h_2f, w_2f, h_2f,
//w_2f, h_2f, w_2f, h_2f
//};
//gl.ViewportArray(0, 4, viewports);
//gl_ViewportIndex
////Copy Paste OVR
//--------------------------------------------------------------------------
//struct StereoTexture
//{
//    GLuint        texIdDepth;

//    ovrHmd              hmd;
//    ovrSwapTextureSet*  TextureSet;
//    GLuint              texId;
//    GLuint              fboId;
//    OVR::Sizei          texSize;

//    StereoTexture(OVR::Sizei size, int sampleCount, int mipLevels, int sampleCount)
//    {
//        QOpenGLFunctions_3_0 funcs;
//        funcs.initializeOpenGLFunctions();
//        OVR_ASSERT(sampleCount <= 1); // The code doesn't currently handle MSAA textures.

//        funcs.glGenTextures(2, &texIdDepth);
//        funcs.glBindTexture(GL_TEXTURE_2D_ARRAY, texIdDepth);
//        funcs.glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//        funcs.glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//        funcs.glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//        funcs.glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

//        GLenum internalFormat = GL_DEPTH_COMPONENT24;
//        GLenum type = GL_UNSIGNED_INT;
//        if (true)//GLE_ARB_depth_buffer_float)
//        {
//            internalFormat = GL_DEPTH_COMPONENT32F;
//            type = GL_FLOAT;
//        }

//        funcs.glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, internalFormat, size.w, size.h, 2, 0, GL_DEPTH_COMPONENT, type, NULL);



//        funcs.glGenTextures(2, &texId);
//        funcs.glBindTexture(GL_TEXTURE_2D_ARRAY, texId);
//        funcs.glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//        funcs.glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//        funcs.glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//        funcs.glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

//        GLenum internalFormat = GL_DEPTH_COMPONENT24;
//        GLenum type = GL_UNSIGNED_INT;
//        if (true)//GLE_ARB_depth_buffer_float)
//        {
//            internalFormat = GL_DEPTH_COMPONENT32F;
//            type = GL_FLOAT;
//        }

//        funcs.glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, internalFormat, size.w, size.h, 2, 0, GL_DEPTH_COMPONENT, type, NULL);
//    }
//    ~StereoTexture()
//    {
//        if (texId)
//        {
//            QOpenGLFunctions_3_0 funcs;
//            funcs.initializeOpenGLFunctions();
//            funcs.glDeleteTextures(1, &texId);
//            texId = 0;
//        }
//    }
//};

struct DepthBuffer
{
    GLuint        texId;

    DepthBuffer(OVR::Sizei size, int sampleCount)
    {
        QOpenGLFunctions_3_0 funcs;
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
            QOpenGLFunctions_3_0 funcs;
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
        QOpenGLFunctions_3_0 funcs;
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
        QOpenGLFunctions_3_0 funcs;
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
        QOpenGLFunctions_3_0 funcs;
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
        QOpenGLFunctions_3_0 funcs;
        funcs.initializeOpenGLFunctions();
        funcs.glBindFramebuffer(GL_FRAMEBUFFER, fboId);
        funcs.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
        funcs.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
    }
};
////
#endif

RenderThread::RenderThread()
    : surface(0)
    , context(0)
    , m_renderFbo(0)
    , m_displayFbo(0)
    , m_vrInitialized(false)
    , m_framecount(0)
    , m_frameIndex(0)
    #ifdef VRMODE
    , m_mirrorTexture(nullptr)
    , m_mirrorFBO( 0 )
    #endif
{
#ifdef VRMODE
    m_eyeRenderTexture[0] = nullptr;
    m_eyeRenderTexture[1] = nullptr;
    m_eyeDepthBuffer[0] = nullptr;
    m_eyeDepthBuffer[1] = nullptr;
#endif
    m_timer.restart();
    QmlMapsRenderViewport::threads << this;
    m_mapsRenderer = new MapsRenderer(&m_renderdata);
#ifdef VRMODE
    connect(renderdata(), &Renderdata::vrmodeChanged, this, [&](bool vrmode) {
        if(vrmode)
        {
            std::shared_ptr<QMetaObject::Connection> con(new QMetaObject::Connection(
                connect(this,
                        &RenderThread::frameFinished,
                        this,
                        &RenderThread::vrThreadMainloop,
                        Qt::QueuedConnection)));
            m_vrMainLoopConnection = con;
            Q_EMIT frameFinished();
        }
        else
        {
            disconnect(*m_vrMainLoopConnection);
            m_vrMainLoopConnection = nullptr;
        }
    });
//    connect(renderdata(), &Renderdata::mirrorDistorsionChanged, this, [&](bool b) {
//        resetMirror();
//    });
//    connect(renderdata(), &Renderdata::mirrorRightEyeChanged, this, [&](bool b) {
//        resetMirror();
//    });
#endif
}

RenderThread::~RenderThread()
{
    delete m_mapsRenderer;
}

void RenderThread::renderNext()
{
    Q_EMIT updated();
    context->makeCurrent(surface);

    if(m_renderFbo)
    {
        if(  (m_renderFbo->size().width()  != m_renderdata.width()
           || m_renderFbo->size().height() != m_renderdata.height()) )
        {
            delete m_renderFbo;
            m_renderFbo = NULL;
            delete m_displayFbo;
            m_displayFbo = NULL;
        }
    }
    if(!m_renderFbo)
    {
        // Initialize the buffers and renderer
        QOpenGLFramebufferObjectFormat format;
        format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
        m_renderFbo = new QOpenGLFramebufferObject(QSize(m_renderdata.width(), m_renderdata.height()), format);
        m_displayFbo = new QOpenGLFramebufferObject(QSize(m_renderdata.width(), m_renderdata.height()), format);

    }
    if( !m_mapsRenderer->isInitialized() )
    {
        initialize();
    }

#ifdef VRMODE
    if(m_renderdata.vrmode())
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
    // We need to flush the contents to the FBO before posting
    // the texture to the other thread, otherwise, we might
    // get unexpected results.
    context->functions()->glFlush();

    m_renderFbo->bindDefault();
    qSwap(m_renderFbo, m_displayFbo);

    Q_EMIT textureReady(m_displayFbo->texture(), QSize(m_renderdata.width(), m_renderdata.height()));
}

void RenderThread::renderNextNonVR()
{
    m_renderFbo->bind();
    context->functions()->glViewport(0, 0, m_renderdata.width(), m_renderdata.height());
    QMatrix4x4 mat;
    QMatrix4x4 view;
    view.setToIdentity();
    float fov = renderdata()->fov();
    float nearClip = 0.5;
    mat.perspective(fov, ((float)m_renderdata.width())/(float)m_renderdata.height(), nearClip, 1000.0 );
    QMatrix4x4 flipY;
    flipY.setToIdentity();
    flipY.data()[4+1] = -1.f;
    mat *= flipY;
    renderdata()->setHeadMatrix(view);
    renderdata()->setHeadDirection(QVector3D(0.0,0.0,1.0));
    renderdata()->setHeadOrientation(view);
    QVector4D vps(m_renderdata.width(), m_renderdata.height(), nearClip, 1000.0f);


    float fovy = fov; // degrees
    float heightOfNearPlane = vps.y() / (2.0*tan(0.5*fovy*3.14159265/180.0));
    //qDebug() << "fov:" << fovy << "2*tan:" << (2.0*tan(0.5*fovy*3.14159265/180.0)) << "hi" << heightOfNearPlane;
    m_mapsRenderer->render(view, mat, vps, heightOfNearPlane);


    m_frametimes[m_framecount][0] = m_timer.elapsed();
    m_framecount++;
    if(m_framecount == FRAMES_AVERAGE_WINDOW)
    {
        m_timer.restart();
        m_framecount = 0;
        double mint=std::numeric_limits<double>::max(), maxt=0.0;
        std::vector<double> sorted(FRAMES_AVERAGE_WINDOW-1);
        for(int i=0;i<(FRAMES_AVERAGE_WINDOW-1);++i)
        {
            auto diff = m_frametimes[i+1][0]-m_frametimes[i][0];
            sorted[i] = diff;
            mint = std::min(diff, mint);
            maxt = std::max(diff, maxt);
        }
        std::sort(sorted.begin(), sorted.end());
        auto median = sorted[FRAMES_AVERAGE_WINDOW/2];
        double avg = m_frametimes[FRAMES_AVERAGE_WINDOW-1][0]-m_frametimes[0][0];
        qDebug() << "FPS:" << static_cast<double>(FRAMES_AVERAGE_WINDOW)/avg
                 << "(t_avg:" << avg/FRAMES_AVERAGE_WINDOW
                 << "(t_med:" << median
                 << ",t_min:" << mint
                 << ",t_max:" << maxt << ")" ;
    }
}

#ifdef VRMODE
void RenderThread::renderNextVR()
{
    //qDebug() << "sync";
    if(!m_vrInitialized)
    {
        initVR();
    }
    //isVisible = (result == ovrSuccess);

    QOpenGLFunctions_3_0 funcs;
    funcs.initializeOpenGLFunctions();
    if(m_renderdata.mirrorEnabled())
    {
        // Blit mirror texture to back buffer
        GLint w;
        GLint h;
        if(m_renderdata.mirrorDistorsion())
        {
            w = m_eyeRenderTexture[1]->texSize.w;
            h = m_eyeRenderTexture[1]->texSize.h;

            funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, m_mirrorFBO);
        }
        else if(m_renderdata.mirrorRightEye())
        {
            w = m_eyeRenderTexture[0]->texSize.w;
            h = m_eyeRenderTexture[0]->texSize.h;
            funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, m_eyeRenderTexture[0]->fboId);
//            funcs.glDeleteFramebuffers(1, &m_mirrorFBO);
//            funcs.glGenFramebuffers(1, &m_mirrorFBO);
//            funcs.glBindFramebuffer(GL_FRAMEBUFFER, m_mirrorFBO);
//            auto tex = reinterpret_cast<ovrGLTexture*>(&m_eyeRenderTexture[0]->TextureSet->Textures[m_eyeRenderTexture[0]->TextureSet->CurrentIndex]);
//            funcs.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex->OGL.TexId, 0);
//            funcs.glBindFramebuffer(GL_FRAMEBUFFER, 0);
//            //funcs.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, dbuffer->texId, 0);
        }
        else
        {
            w = m_mirrorTexture->OGL.Header.TextureSize.w;
            h = m_mirrorTexture->OGL.Header.TextureSize.h;
            funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, m_eyeRenderTexture[1]->fboId);
//            w = m_eyeRenderTexture[1]->texSize.w;
//            h = m_eyeRenderTexture[1]->texSize.h;
//            funcs.glDeleteFramebuffers(1, &m_mirrorFBO);
//            funcs.glGenFramebuffers(1, &m_mirrorFBO);
//            funcs.glBindFramebuffer(GL_FRAMEBUFFER, m_mirrorFBO);
//            auto tex = reinterpret_cast<ovrGLTexture*>(&m_eyeRenderTexture[1]->TextureSet->Textures[m_eyeRenderTexture[1]->TextureSet->CurrentIndex]);
//            funcs.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex->OGL.TexId, 0);
//            funcs.glBindFramebuffer(GL_FRAMEBUFFER, 0);
            //funcs.glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, dbuffer->texId, 0);
        }

        //funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, m_mirrorFBO);


        funcs.glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_renderFbo->handle());
        funcs.glBlitFramebuffer(0, 0, w, h,
                          0, 0, w, h,
                          GL_COLOR_BUFFER_BIT, GL_NEAREST);
        funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    }
//    SwapBuffers(Platform.hDC);

    m_renderFbo->bind();
}

void RenderThread::vrThreadMainloop()
{

    //qDebug() << "frame";
    if(!m_renderdata.running() || !m_renderdata.vrmode())
    {
        Q_EMIT frameFinished();
        return;
    }
    m_frameIndex++;
    if(!m_vrInitialized)
    {
        initVR();
    }

    static float Yaw(3.141592f);

    // Keyboard inputs to adjust player position
//    static OVR::Vector3f Pos2(0.0f,1.6f,-5.0f);
//    Pos2.y = ovr_GetFloat(HMD, OVR_KEY_EYE_HEIGHT, Pos2.y);

    // Get eye poses, feeding in correct IPD offset
    ovrVector3f               ViewOffset[2] = { m_eyeRenderDesc[0].HmdToEyeViewOffset,
                                                m_eyeRenderDesc[1].HmdToEyeViewOffset };
    ovrPosef                  EyeRenderPose[2];

    double           ftiming = ovr_GetPredictedDisplayTime(m_HMD, m_frameIndex);
    // Keeping sensorSampleTime as close to ovr_GetTrackingState as possible - fed into the layer
    double           sensorSampleTime = ovr_GetTimeInSeconds();
    m_frametimes[m_framecount][2] = m_timer.elapsed();
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
            OVR::Vector3f shiftedEyePos = /*Pos2 +*/ rollPitchYaw.Transform(EyeRenderPose[eye].Position);

            OVR::Matrix4f view = OVR::Matrix4f::LookAtRH(shiftedEyePos, shiftedEyePos + finalForward, finalUp);
            float nearClip = 0.2f;
            OVR::Matrix4f proj = ovrMatrix4f_Projection(m_hmdDesc.DefaultEyeFov[eye], nearClip, 1000.0f, ovrProjection_RightHanded);
            QMatrix4x4 qview(&view.M[0][0]);
            QMatrix4x4 qproj(&proj.M[0][0]);

            QMatrix4x4 qorientation(&finalRollPitchYaw.M[0][0]);//QQuaternion(orientation.w, orientation.x, orientation.y, orientation.z));
            QVector3D qdir(finalForward.x, finalForward.y, finalForward.z);
            if(eye == 1)
            {
                renderdata()->setHeadMatrix(qview);
                renderdata()->setHeadDirection(qdir);
                renderdata()->setHeadOrientation(qorientation);
            }
            // Render world
            //roomScene->Render(view, proj);
            //context->functions()->glViewport(0, 0, m_size.width(), m_size.height());
            QVector4D vps(m_eyeRenderTexture[eye]->texSize.w, m_eyeRenderTexture[eye]->texSize.h, nearClip, 1000.0f);

            float top = m_hmdDesc.DefaultEyeFov[eye].UpTan;
            float bottom = m_hmdDesc.DefaultEyeFov[eye].DownTan;
            //float left = nearClip * m_hmdDesc.DefaultEyeFov[eye].LeftTan;
            //float right = nearClip * m_hmdDesc.DefaultEyeFov[eye].RightTan;
            //qDebug() << "t" << top << "b" << bottom << "r" << right << "l" << left << "2*tan:" << (top+bottom) << "hi:" << vps.y()/(top+bottom);
            m_mapsRenderer->render(qview, qproj, vps,  vps.y() / (top+bottom));

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
    ovrResult result = ovr_SubmitFrame(m_HMD, m_frameIndex, &viewScaleDesc, &layers, 1);

    m_frametimes[m_framecount][0] = m_timer.elapsed();
    m_framecount++;
    if(m_framecount == FRAMES_AVERAGE_WINDOW)
    {
        m_timer.restart();
        m_framecount = 0;
        double mint=std::numeric_limits<double>::max(), maxt=0.0;
        std::vector<double> sorted(FRAMES_AVERAGE_WINDOW-1);
        for(int i=0;i<(FRAMES_AVERAGE_WINDOW-1);++i)
        {
            auto diff = m_frametimes[i+1][0]-m_frametimes[i][0];
            sorted[i] = diff;
            mint = std::min(diff, mint);
            maxt = std::max(diff, maxt);
        }
        std::sort(sorted.begin(), sorted.end());
        auto median = sorted[FRAMES_AVERAGE_WINDOW/2];
        double avg = m_frametimes[FRAMES_AVERAGE_WINDOW-1][0]-m_frametimes[0][0];
        qDebug() << "FPS:" << static_cast<double>(FRAMES_AVERAGE_WINDOW)/avg
                 << "(t_avg:" << avg/FRAMES_AVERAGE_WINDOW
                 << "(t_med:" << median
                 << ",t_min:" << mint
                 << ",t_max:" << maxt << ")" ;
    }
    // exit the rendering loop if submit returns an error, will retry on ovrError_DisplayLost
    if (!OVR_SUCCESS(result))
    {
        qDebug() << "err submit frame";
        Q_EMIT frameFinished();
        return;
    }
    Q_EMIT frameFinished();
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

void RenderThread::initialize()
{
#ifndef RELEASE
        if ( m_logger.initialize() ) {
            connect( &m_logger, &QOpenGLDebugLogger::messageLogged,
                        this, &RenderThread::onMessageLogged,
                        Qt::DirectConnection );
            m_logger.startLogging( QOpenGLDebugLogger::SynchronousLogging );
            m_logger.enableMessages();
            QVector<uint> disabledMessages;
            disabledMessages.push_back(131185); //Buffer Detailed Info
            disabledMessages.push_back(131204); // Texture has mipmaps, filter is inconsistent with mipmaps (framebuffer?)
            //disabledMessages.push_back(131218);
            disabledMessages.push_back(131184);
            m_logger.disableMessages(disabledMessages);
        }
#endif
//        if( m_renderdata.entitydata() != NULL && m_renderdata.entitydata()->getEntitydata() != NULL)
//        {
//            m_mapsRenderer->setEntitydata(m_renderdata.entitydata()->getEntitydata());
//        }
//        else if(!m_renderdata.filename().isEmpty()) // elseif workaround
//        {
//            m_mapsRenderer->setFilename(m_renderdata.filename());
//        }
        m_mapsRenderer->initialize();
        m_renderdata.setRunning(true);
}

//void RenderThread::setWidth(qreal width)
//{
//    if (m_size.width() == width)
//        return;

//    m_size.setWidth(width);
//    m_mapsRenderer->setScreenSize(m_size);
//    if(m_vrInitialized) resetMirror();
//    Q_EMIT widthChanged(width);
//}

//void RenderThread::setHeight(qreal height)
//{
//    if (m_size.height() == height)
//        return;

//    m_size.setHeight(height);
//    m_mapsRenderer->setScreenSize(m_size);
//    if(m_vrInitialized) resetMirror();
//    Q_EMIT heightChanged(height);
//}

//void RenderThread::setMatrix(QMatrix4x4 matrix)
//{
//    if (m_matrix == matrix)
//        return;

//    m_matrix = matrix;
//    if(m_mapsRenderer)
//        m_mapsRenderer->setMatrix( matrix );
//    Q_EMIT matrixChanged(matrix);
//}

//void RenderThread::setEntitydata(QmlEntitydata *entitydata)
//{
//    if (m_entitydata == entitydata)
//        return;

//    m_entitydata = entitydata;
//    m_mapsRenderer->setEntitydata(m_entitydata->getEntitydata());
//    Q_EMIT entitydataChanged(entitydata);
//}

//void RenderThread::setVrmode(bool vrmode)
//{
//#ifndef VRMODE
//    assert(!vrmode);
//#endif
//    if (m_vrmode == vrmode)
//        return;

//    m_vrmode = vrmode;
//    if(m_vrmode)
//    {
//        qDebug() << "con";
//        m_vrMainLoopConnection = connect(this, &RenderThread::frameFinished, this, &RenderThread::vrThreadMainloop, Qt::QueuedConnection);
//        Q_EMIT frameFinished();
//    }
//    else
//    {
//        qDebug() << "discon";
//        disconnect(m_vrMainLoopConnection);
//    }
//    Q_EMIT vrmodeChanged(vrmode);
//}

//void RenderThread::setMirrorEnabled(bool mirrorEnabled)
//{
//    if (m_mirrorEnabled == mirrorEnabled)
//        return;

//    m_mirrorEnabled = mirrorEnabled;
//    Q_EMIT mirrorEnabledChanged(mirrorEnabled);
//}

//void RenderThread::setMirrorDistorsion(bool mirrorDistorsion)
//{
//    if (m_mirrorDistorsion == mirrorDistorsion)
//        return;

//    m_mirrorDistorsion = mirrorDistorsion;
//#ifdef VRMODE
//    if(m_mirrorDistorsion)
//    {
//        QOpenGLFunctions_3_0 funcs;
//        funcs.initializeOpenGLFunctions();
//        funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, m_mirrorFBO);
//        funcs.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_mirrorTexture->OGL.TexId, 0);
//        funcs.glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
//        funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
//    }
//#endif
//    Q_EMIT mirrorDistorsionChanged(mirrorDistorsion);
//}

//void RenderThread::setMirrorRightEye(bool mirrorRightEye)
//{
//    if (m_mirrorRightEye == mirrorRightEye)
//        return;

//    m_mirrorRightEye = mirrorRightEye;
//    if(!m_mirrorDistorsion)
//    {
////        if(m_mirrorRightEye)
////        {
////            QOpenGLFunctions_3_0 funcs;
////            funcs.initializeOpenGLFunctions();
////            funcs.glDeleteFramebuffers(1, &m_mirrorFBO);
////            funcs.glGenFramebuffers(1, &m_mirrorFBO);
////            funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, m_mirrorFBO);
////            funcs.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_eyeRenderTexture[1]->texId, 0);
////            //funcs.glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
////            funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
////        }
////        else
////        {
////            QOpenGLFunctions_3_0 funcs;
////            funcs.initializeOpenGLFunctions();
////            funcs.glDeleteFramebuffers(1, &m_mirrorFBO);
////            funcs.glGenFramebuffers(1, &m_mirrorFBO);
////            funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, m_mirrorFBO);
////            funcs.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_eyeRenderTexture[0]->texId, 0);
////            //funcs.glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
////            funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
////        }
//    }
//    Q_EMIT mirrorRightEyeChanged(mirrorRightEye);
//}

//void RenderThread::setPointSize(qreal pointSize)
//{
//    if (m_pointSize == pointSize)
//        return;

//    m_pointSize = pointSize;
//    m_mapsRenderer->setPointSize(pointSize);
//    Q_EMIT pointSizeChanged(pointSize);
//}

//void RenderThread::setFilename(QString filename)
//{
//    if (m_filename == filename)
//        return;

//    m_filename = filename;
//    m_mapsRenderer->setFilename(filename);
//    Q_EMIT filenameChanged(filename);
//}

//void RenderThread::setDistanceDetail(qreal distanceDetail)
//{
//    if (m_distanceDetail == distanceDetail)
//        return;

//    m_distanceDetail = distanceDetail;
//    m_mapsRenderer->setDistanceDetail(distanceDetail);
//    Q_EMIT distanceDetailChanged(distanceDetail);
//}

//void RenderThread::setRunning(bool running)
//{
//    if (m_running == running)
//        return;

//    m_running = running;
//    Q_EMIT runningChanged(running);
//}

void RenderThread::resetMirror()
{
#ifdef VRMODE
    if(!m_vrInitialized) return;
    context->makeCurrent(surface);
    //ovrSizei windowSize = { m_hmdDesc.Resolution.w / 2, m_hmdDesc.Resolution.h / 2 };
    ovrSizei windowSize = { m_renderdata.width(), m_renderdata.height() };

    if(m_mirrorTexture)
    {
        //return; //TODO: does not yet work
        ovr_DestroyMirrorTexture(m_HMD, reinterpret_cast<ovrTexture*>(&m_mirrorTexture));
        m_mirrorTexture = nullptr;
    }
    // Create mirror texture and an FBO used to copy mirror texture to back buffer
    ovrResult result = ovr_CreateMirrorTextureGL(m_HMD, GL_SRGB8_ALPHA8, windowSize.w, windowSize.h, reinterpret_cast<ovrTexture**>(&m_mirrorTexture));
    if (!OVR_SUCCESS(result))
    {
        std::cout << "Failed to create mirror" << std::endl;
        if (true) return;// goto Done;
        //VALIDATE(false, "Failed to create mirror texture.");
    }
    std::cout << "New Width:" << m_mirrorTexture->OGL.Header.TextureSize.w << " H: " << m_mirrorTexture->OGL.Header.TextureSize.h << std::endl;


    QOpenGLFunctions_3_0 funcs;
    funcs.initializeOpenGLFunctions();
    // Configure the mirror read buffer
    if(m_mirrorFBO)
    {
        funcs.glDeleteFramebuffers(1, &m_mirrorFBO);
    }
    funcs.glGenFramebuffers(1, &m_mirrorFBO);
    funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, m_mirrorFBO);
//    if(m_renderdata.mirrorDistorsion())
//    {
        funcs.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_mirrorTexture->OGL.TexId, 0);
        funcs.glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
//    }
//    else
//    {
//        if(m_renderdata.mirrorRightEye())
//        {
//            funcs.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_eyeRenderTexture[1]->texId, 0);
//            //funcs.glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
//        }
//        else
//        {
//            funcs.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_eyeRenderTexture[0]->texId, 0);
//            //funcs.glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
//        }
//    }
    funcs.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
#endif
}

//void RenderThread::setHeadOrientation(QMatrix4x4 headOrientation)
//{
//    //    if (m_headOrientation == headOrientation)
//    //        return;

//    //    m_headOrientation = headOrientation;
//    Q_EMIT headOrientationChanged(headOrientation);
//}

//void RenderThread::setHeadMatrix(QMatrix4x4 headMatrix)
//{
////    if (m_headMatrix == headMatrix)
////        return;

////    m_headMatrix = headMatrix;
//    Q_EMIT headMatrixChanged(headMatrix);
//}

//void RenderThread::setHeadDirection(QVector3D headDirection)
//{
////    if (m_headDirection == headDirection)
////        return;

////    m_headDirection = headDirection;
//    Q_EMIT headDirectionChanged(headDirection);
//}

void RenderThread::onMessageLogged(QOpenGLDebugMessage message)
{
    qDebug() << message;
}

#ifdef VRMODE
void RenderThread::initVR()
{
    qDebug() << "init";
    ovrResult result = ovr_Initialize(nullptr);
    if (!OVR_SUCCESS(result))
        return;
    ovrGraphicsLuid luid;
    result = ovr_Create(&m_HMD, &luid);
    if (!OVR_SUCCESS(result))
        return;

    m_hmdDesc = ovr_GetHmdDesc(m_HMD);


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

    m_vrInitialized = true;
    resetMirror();

    m_eyeRenderDesc[0] = ovr_GetRenderDesc(m_HMD, ovrEye_Left, m_hmdDesc.DefaultEyeFov[0]);
    m_eyeRenderDesc[1] = ovr_GetRenderDesc(m_HMD, ovrEye_Right, m_hmdDesc.DefaultEyeFov[1]);

    // Turn off vsync to let the compositor do its magic
//    wglSwapIntervalEXT(0);


//    Done:
    return;
}
#endif

//QmlEntitydata *RenderThread::entitydata() const
//{
//    return m_entitydata;
//}

void RenderThread::reload()
{

}
