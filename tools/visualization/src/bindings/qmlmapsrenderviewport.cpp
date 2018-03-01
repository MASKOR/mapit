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

#include "upns/ui/bindings/qmlmapsrenderviewport.h"

#include <QMutex>
#include <QThread>

#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QGuiApplication>
#include <QOffscreenSurface>

#include <QQuickWindow>
#include <QSGSimpleTextureNode>

#include "render/renderthread.h"
#include "render/texturenode.h"

QList<QThread *> QmlMapsRenderViewport::threads;

QmlMapsRenderViewport::QmlMapsRenderViewport()
    : m_renderThread(0)
{
    setFlag(ItemHasContents, true);
    m_renderThread = new RenderThread();

    m_renderThread->renderdata()->connectReadInputFrom(renderdata());
    m_renderThread->renderdata()->connectWriteOutputTo(renderdata());
    m_renderThread->renderdata()->connectReadInputWidthHeightFrom(this);

    connect(renderdata(), &Renderdata::entitydataChanged, this, [&](QmlEntitydata *entitydata) {

    });
    connect(renderdata(), &Renderdata::matrixChanged, this, &QmlMapsRenderViewport::finalTransformChanged);
    connect(renderdata(), &Renderdata::headOrientationChanged, this, &QmlMapsRenderViewport::finalTransformChanged);
}

QMatrix4x4 QmlMapsRenderViewport::finalTransform() const
{
    return m_renderdata.headOrientation()*m_renderdata.matrix();//QMatrix4x4((m_renderdata.headOrientation()*m_renderdata.matrix()).normalMatrix());
}

void QmlMapsRenderViewport::ready()
{
    m_renderThread->surface = new QOffscreenSurface();
    m_renderThread->surface->setFormat(m_renderThread->context->format());
    m_renderThread->surface->create();

    m_renderThread->moveToThread(m_renderThread);

    connect(window(), SIGNAL(sceneGraphInvalidated()), m_renderThread, SLOT(shutDown()), Qt::QueuedConnection);

    m_renderThread->start();
    QQuickItem::update();
}

QSGNode *QmlMapsRenderViewport::updatePaintNode(QSGNode *oldNode, UpdatePaintNodeData *)
{
    TextureNode *node = static_cast<TextureNode *>(oldNode);
    if (!m_renderThread->context) {
        QOpenGLContext *current = window()->openglContext();
        // Some GL implementations requres that the currently bound context is
        // made non-current before we set up sharing, so we doneCurrent here
        // and makeCurrent down below while setting up our own context.
        current->doneCurrent();

        m_renderThread->context = new QOpenGLContext();
        m_renderThread->context->setFormat(current->format());
        m_renderThread->context->setShareContext(current);
        m_renderThread->context->create();
        m_renderThread->context->moveToThread(m_renderThread);

        current->makeCurrent(window());

        QMetaObject::invokeMethod(this, "ready");
        return 0;
    }

    if (!node) {
        node = new TextureNode(window());

        /* Set up connections to get the production of FBO textures in sync with vsync on the
         * rendering thread.
         *
         * When a new texture is ready on the rendering thread, we use a direct connection to
         * the texture node to let it know a new texture can be used. The node will then
         * emit pendingNewTexture which we bind to QQuickWindow::update to schedule a redraw.
         *
         * When the scene graph starts rendering the next frame, the prepareNode() function
         * is used to update the node with the new texture. Once it completes, it emits
         * textureInUse() which we connect to the FBO rendering thread's renderNext() to have
         * it start producing content into its current "back buffer".
         *
         * This FBO rendering pipeline is throttled by vsync on the scene graph rendering thread.
         */
        connect(m_renderThread, SIGNAL(textureReady(int,QSize)), node, SLOT(newTexture(int,QSize)), Qt::DirectConnection);
        connect(node, SIGNAL(pendingNewTexture()), window(), SLOT(update()), Qt::QueuedConnection);
        connect(window(), SIGNAL(beforeRendering()), node, SLOT(prepareNode()), Qt::DirectConnection);
        connect(node, SIGNAL(textureInUse()), m_renderThread, SLOT(renderNext()), Qt::QueuedConnection);
        connect(m_renderThread, SIGNAL(updated()), this, SLOT(emitFrame()), Qt::QueuedConnection);

        // Get the production of FBO textures started..
        QMetaObject::invokeMethod(m_renderThread, "renderNext", Qt::QueuedConnection);
    }

    node->setRect(boundingRect());

    return node;
}

//void QmlMapsRenderViewport::reload()
//{
//    //TODO: Q_EMIT needsReload();
//}

//void QmlMapsRenderViewport::setEntitydata(QmlEntitydata *entitydata)
//{
//    if (m_entitydata == entitydata)
//        return;
//    if(m_entitydata)
//    {
//        if(m_connectionToEntitydata != NULL)
//        {
//            disconnect(*m_connectionToEntitydata);
//            m_connectionToEntitydata = NULL;
//        }
//        //            disconnect(m_entitydata, &QmlEntitydata::updated, this, &QmlMapsRenderViewport::entitydataChanged);
//    }
//    m_entitydata = entitydata;
//    if(m_entitydata)
//    {
//        std::shared_ptr<QMetaObject::Connection> con( new QMetaObject::Connection(
//                                                                  connect(m_entitydata,
//                                                                          &QmlEntitydata::updated,
//                                                                          m_renderThread,
//                                                                          [&](){m_renderThread->setEntitydata(m_entitydata);})));
//        m_connectionToEntitydata = con;
//        //            connect(m_entitydata, &QmlEntitydata::updated, this, &QmlMapsRenderViewport::entitydataChanged);
//        //            connect(this, &QmlEntitydata::updated, this, &RenderThread::updated);
//    }
//    Q_EMIT entitydataChanged(entitydata);
//}
//void QmlMapsRenderViewport::setVrmode(bool vrmode)
//{
//#ifdef VRMODE
//    if (m_vrmode == vrmode)
//        return;
//    m_vrmode = vrmode;
//    if(m_renderThread)
//    {
//        m_renderThread->setVrmode(vrmode);
//    }
//    assert(!vrmode);
//    Q_EMIT vrmodeChanged(vrmode);
//#endif
//}

void QmlMapsRenderViewport::emitFrame()
{
    Q_EMIT frame();
}

//void QmlMapsRenderViewport::setHeadDirection(QVector3D headDirection)
//{
//    if (m_headDirection == headDirection)
//        return;

//    m_headDirection = headDirection;
//    Q_EMIT headDirectionChanged(headDirection);
//}

//void QmlMapsRenderViewport::setHeadOrientation(QMatrix4x4 headOrientation)
//{
//    if (m_headOrientation == headOrientation)
//        return;

//    m_headOrientation = headOrientation;
//    Q_EMIT headOrientationChanged(headOrientation);
//}

//void QmlMapsRenderViewport::setHeadMatrix(QMatrix4x4 headMatrix)
//{
//    if (m_headMatrix == headMatrix)
//        return;

//    m_headMatrix = headMatrix;
//    Q_EMIT headMatrixChanged(headMatrix);
//}

//void QmlMapsRenderViewport::setRunning(bool running)
//{
//    if (m_running == running)
//        return;

//    m_running = running;
//    Q_EMIT runningChanged(running);
//}

#include "qmlmapsrenderviewport.moc"


