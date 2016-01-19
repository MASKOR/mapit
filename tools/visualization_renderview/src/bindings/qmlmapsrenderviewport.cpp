#include "bindings/qmlmapsrenderviewport.h"

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
    : m_renderThread(0)//,
      //m_connectionToEntityData(NULL)
{
    setFlag(ItemHasContents, true);
    m_renderThread = new RenderThread(QSize(width(), height()));
    connect(this, &QmlMapsRenderViewport::entitydataChanged, m_renderThread, &RenderThread::setEntitydata);
    connect(this, &QQuickItem::widthChanged, m_renderThread, [&](){m_renderThread->setWidth(width());});
    connect(this, &QQuickItem::heightChanged, m_renderThread, [&](){m_renderThread->setHeight(height());});
    connect(this, &QmlMapsRenderViewport::needsReload, m_renderThread, &RenderThread::reload);
    connect(this, &QmlMapsRenderViewport::matrixChanged, m_renderThread, &RenderThread::setMatrix);
}

void QmlMapsRenderViewport::ready()
{
    m_renderThread->surface = new QOffscreenSurface();
    m_renderThread->surface->setFormat(m_renderThread->context->format());
    m_renderThread->surface->create();

    m_renderThread->moveToThread(m_renderThread);

    connect(window(), SIGNAL(sceneGraphInvalidated()), m_renderThread, SLOT(shutDown()), Qt::QueuedConnection);

    m_renderThread->start();
    update();
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

        // Get the production of FBO textures started..
        QMetaObject::invokeMethod(m_renderThread, "renderNext", Qt::QueuedConnection);
    }

    node->setRect(boundingRect());

    return node;
}

void QmlMapsRenderViewport::reload()
{
    Q_EMIT needsReload();
}

#include "qmlmapsrenderviewport.moc"

