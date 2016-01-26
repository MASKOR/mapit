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
    Q_PROPERTY(QMatrix4x4 headMatrix READ headMatrix NOTIFY headMatrixChanged)
    Q_PROPERTY(QMatrix4x4 headOrientation READ headOrientation NOTIFY headOrientationChanged)

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

    QMatrix4x4 headMatrix() const
    {
        return m_headMatrix;
    }

    QMatrix4x4 headOrientation() const
    {
        return m_headOrientation;
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

Q_SIGNALS:
    void matrixChanged(QMatrix4x4 matrix);

    void entitydataChanged(QmlEntitydata * entitydata);
    void updated(upns::upnsSharedPointer<upns::AbstractEntityData> entitydata);

    void needsReload();
    void vrmodeChanged(bool vrmode);

    void headDirectionChanged(QVector3D headDirection);
    void headMatrixChanged(QMatrix4x4 headMatrix);
    void headOrientationChanged(QMatrix4x4 headOrientation);

protected:
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);

private Q_SLOTS:
    void setHeadDirection(QVector3D headDirection);
    void setHeadOrientation(QMatrix4x4 headOrientation);
    void setHeadMatrix(QMatrix4x4 headMatrix);

private:
    RenderThread *m_renderThread;
    QMatrix4x4 m_matrix;
    QmlEntitydata * m_entitydata;
    upns::upnsSharedPointer<QMetaObject::Connection> m_connectionToEntityData;
    bool m_vrmode;
    QVector3D m_headDirection;
    QMatrix4x4 m_headMatrix;
    QMatrix4x4 m_headOrientation;
};

#endif // MapsRenderer_H
