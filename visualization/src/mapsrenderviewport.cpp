#include "mapsrenderviewport.h"

#include <QMutex>
#include <QThread>

#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QGuiApplication>
#include <QOffscreenSurface>

#include <QQuickWindow>
#include <QSGSimpleTextureNode>

#include "mapsrenderer.h"

QList<QThread *> MapsRenderViewport::threads;

/*
 * The render thread shares a context with the scene graph and will
 * render into two separate FBOs, one to use for display and one
 * to use for rendering
 */
class RenderThread : public QThread
{
    Q_OBJECT
    Q_PROPERTY(QString mapId READ mapId WRITE setMapId NOTIFY mapIdChanged)
    Q_PROPERTY(upns::MapManager *mapManager READ mapManager WRITE setMapManager NOTIFY mapManagerChanged)
    Q_PROPERTY(qreal width READ width WRITE setWidth NOTIFY widthChanged)
    Q_PROPERTY(qreal height READ height WRITE setHeight NOTIFY heightChanged)
public:
    RenderThread(const QSize &size)
        : surface(0)
        , context(0)
        , m_renderFbo(0)
        , m_displayFbo(0)
        , m_mapsRenderer(0)
        , m_size(size)
        , m_mapManager(0)
        , m_mapId("")
    {
        MapsRenderViewport::threads << this;
    }

    QOffscreenSurface *surface;
    QOpenGLContext *context;

    upns::MapManager * mapManager() const
    {
        return m_mapManager;
    }

    QString mapId() const
    {
        return m_mapId;
    }

    qreal width() const
    {
        return m_size.width();
    }

    qreal height() const
    {
        return m_size.height();
    }

public Q_SLOTS:
    void renderNext()
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
        if( !m_mapsRenderer )
        {
            m_mapsRenderer = new MapsRenderer();
            m_mapsRenderer->setMapId( mapId().toULongLong() );
            m_mapsRenderer->setMapmanager( mapManager() );
            m_mapsRenderer->initialize();
        }

        m_renderFbo->bind();
        context->functions()->glViewport(0, 0, m_size.width(), m_size.height());

        m_mapsRenderer->render();

        // We need to flush the contents to the FBO before posting
        // the texture to the other thread, otherwise, we might
        // get unexpected results.
        context->functions()->glFlush();

        m_renderFbo->bindDefault();
        qSwap(m_renderFbo, m_displayFbo);

        Q_EMIT textureReady(m_displayFbo->texture(), m_size);
    }

    void shutDown()
    {
        context->makeCurrent(surface);
        delete m_renderFbo;
        delete m_displayFbo;
        delete m_mapsRenderer;
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

    void setMapManager(upns::MapManager * mapManager)
    {
        if (m_mapManager == mapManager)
            return;

        m_mapManager = mapManager;
        if(m_mapsRenderer)
            m_mapsRenderer->setMapmanager( mapManager );
        Q_EMIT mapManagerChanged(mapManager);
    }

    void setMapId(QString mapId)
    {
        if (m_mapId == mapId)
            return;

        m_mapId = mapId;
        if(m_mapsRenderer)
            m_mapsRenderer->setMapId( mapId.toULongLong() );
        Q_EMIT mapIdChanged(mapId);
    }

    void setWidth(qreal width)
    {
        if (m_size.width() == width)
            return;

        m_size.setWidth(width);
        Q_EMIT widthChanged(width);
    }

    void setHeight(qreal height)
    {
        if (m_size.height() == height)
            return;

        m_size.setHeight(height);
        Q_EMIT heightChanged(height);
    }

Q_SIGNALS:
    void textureReady(int id, const QSize &size);

    void mapManagerChanged(upns::MapManager * mapManager);

    void mapIdChanged(QString mapId);

    void widthChanged(qreal width);

    void heightChanged(qreal height);

private:
    QOpenGLFramebufferObject *m_renderFbo;
    QOpenGLFramebufferObject *m_displayFbo;

    MapsRenderer *m_mapsRenderer;
    QSize m_size;
    upns::MapManager * m_mapManager;
    QString m_mapId;
};

class TextureNode : public QObject, public QSGSimpleTextureNode
{
    Q_OBJECT

public:
    TextureNode(QQuickWindow *window)
        : m_id(0)
        , m_size(0, 0)
        , m_texture(0)
        , m_window(window)
    {
        // Our texture node must have a texture, so use the default 0 texture.
        m_texture = m_window->createTextureFromId(0, QSize(1, 1));
        setTexture(m_texture);
        setFiltering(QSGTexture::Linear);
    }

    ~TextureNode()
    {
        delete m_texture;
    }

Q_SIGNALS:
    void textureInUse();
    void pendingNewTexture();

public Q_SLOTS:

    // This function gets called on the FBO rendering thread and will store the
    // texture id and size and schedule an update on the window.
    void newTexture(int id, const QSize &size) {
        m_mutex.lock();
        m_id = id;
        m_size = size;
        m_mutex.unlock();

        // We cannot call QQuickWindow::update directly here, as this is only allowed
        // from the rendering thread or GUI thread.
        Q_EMIT pendingNewTexture();
    }

    // Before the scene graph starts to render, we update to the pending texture
    void prepareNode() {
        m_mutex.lock();
        int newId = m_id;
        QSize size = m_size;
        m_id = 0;
        m_mutex.unlock();
        if (newId) {
            delete m_texture;
            // note: include QQuickWindow::TextureHasAlphaChannel if the rendered content
            // has alpha.
            m_texture = m_window->createTextureFromId(newId, size);
            setTexture(m_texture);

            markDirty(DirtyMaterial);

            // This will notify the rendering thread that the texture is now being rendered
            // and it can start rendering to the other one.
            Q_EMIT textureInUse();
        }
    }

private:

    int m_id;
    QSize m_size;

    QMutex m_mutex;

    QSGTexture *m_texture;
    QQuickWindow *m_window;
};

MapsRenderViewport::MapsRenderViewport()
    : m_renderThread(0)
{
    setFlag(ItemHasContents, true);
    m_renderThread = new RenderThread(QSize(width(), height()));
    connect(this, &MapsRenderViewport::mapManagerChanged, m_renderThread, [&]() {
            m_renderThread->setMapManager(mapManager()?mapManager()->getMapManager():NULL);
    });
    connect(this, &MapsRenderViewport::mapIdChanged, m_renderThread, &RenderThread::setMapId);
    connect(this, &QQuickItem::widthChanged, m_renderThread, [&](){m_renderThread->setWidth(width());});
    connect(this, &QQuickItem::heightChanged, m_renderThread, [&](){m_renderThread->setHeight(height());});
}

void MapsRenderViewport::ready()
{
    m_renderThread->surface = new QOffscreenSurface();
    m_renderThread->surface->setFormat(m_renderThread->context->format());
    m_renderThread->surface->create();

    m_renderThread->moveToThread(m_renderThread);

    connect(window(), SIGNAL(sceneGraphInvalidated()), m_renderThread, SLOT(shutDown()), Qt::QueuedConnection);

    m_renderThread->start();
    update();
}

QSGNode *MapsRenderViewport::updatePaintNode(QSGNode *oldNode, UpdatePaintNodeData *)
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

#include "mapsrenderviewport.moc"
