#ifndef MapsRenderer_H
#define MapsRenderer_H

#include <QQuickItem>
#include <QMatrix4x4>
#include "bindings/qmlentitydata.h"

class RenderThread;

class QmlMapsRenderViewport : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(QmlEntitydata *entitydata READ entitydata WRITE setEntitydata NOTIFY entitydataChanged)
    Q_PROPERTY(QMatrix4x4 matrix READ matrix WRITE setMatrix NOTIFY matrixChanged)
    Q_PROPERTY(bool vrmode READ vrmode WRITE setVrmode NOTIFY vrmodeChanged)
    Q_PROPERTY(QVector3D headDirection READ headDirection NOTIFY headDirectionChanged)

public:
    QmlMapsRenderViewport();

    static QList<QThread *> threads;

    QMatrix4x4 matrix() const
    {
        return m_matrix;
    }

    QmlEntitydata * entitydata() const
    {
        return m_entitydata;
    }

    bool vrmode() const
    {
        return m_vrmode;
    }

    QVector3D headDirection() const
    {
        return m_headDirection;
    }

public Q_SLOTS:
    void ready();
    void reload();

    void setMatrix(QMatrix4x4 matrix)
    {
        if (m_matrix == matrix)
            return;

        m_matrix = matrix;
        Q_EMIT matrixChanged(matrix);
    }

    void setEntitydata(QmlEntitydata * entitydata);

    void setVrmode(bool vrmode);

    void setHeadDirection(QVector3D headDirection)
    {
        if (m_headDirection == headDirection)
            return;

        m_headDirection = headDirection;
        Q_EMIT headDirectionChanged(headDirection);
    }

Q_SIGNALS:

    void matrixChanged(QMatrix4x4 matrix);

    void entitydataChanged(QmlEntitydata * entitydata);
    void updated(upns::upnsSharedPointer<upns::AbstractEntityData> entitydata);

    void needsReload();
    void vrmodeChanged(bool vrmode);

    void headDirectionChanged(QVector3D headDirection);

protected:
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);

private:
    RenderThread *m_renderThread;
    QMatrix4x4 m_matrix;
    QmlEntitydata * m_entitydata;
    upns::upnsSharedPointer<QMetaObject::Connection> m_connectionToEntityData;
    bool m_vrmode;
    QVector3D m_headDirection;
};

#endif // MapsRenderer_H
