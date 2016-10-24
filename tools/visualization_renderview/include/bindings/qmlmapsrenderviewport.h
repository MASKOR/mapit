#ifndef MapsRenderer_H
#define MapsRenderer_H

#include <QQuickItem>
#include <QMatrix4x4>
#include "bindings/qmlentitydata.h"
#include "renderdata.h"
#include <QTransform>

class RenderThread;

class QmlMapsRenderViewport : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(Renderdata *renderdata READ renderdata NOTIFY renderdataChanged)
    Q_PROPERTY(QMatrix4x4 finalTransform READ finalTransform NOTIFY finalTransformChanged)
public:
    QmlMapsRenderViewport();

    static QList<QThread *> threads;

    Renderdata *renderdata()
    {
        return &m_renderdata;
    }

    QMatrix4x4 finalTransform() const;

public Q_SLOTS:
    void ready();
    //TODO void reload();
    void emitFrame();

Q_SIGNALS:
    void frame();

    void renderdataChanged(Renderdata * renderdata);

    void finalTransformChanged();

protected:
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);

private Q_SLOTS:

private:
    RenderThread *m_renderThread;

    upns::upnsSharedPointer<QMetaObject::Connection> m_connectionToEntitydata;
    Renderdata m_renderdata;
};

#endif // MapsRenderer_H
