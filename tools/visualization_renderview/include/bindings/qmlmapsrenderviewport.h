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
    Q_PROPERTY(QString filename READ filename WRITE setFilename NOTIFY filenameChanged)
    Q_PROPERTY(QMatrix4x4 matrix READ matrix WRITE setMatrix NOTIFY matrixChanged)
    Q_PROPERTY(bool vrmode READ vrmode WRITE setVrmode NOTIFY vrmodeChanged)
    Q_PROPERTY(QVector3D headDirection READ headDirection NOTIFY headDirectionChanged)
    Q_PROPERTY(QMatrix4x4 headMatrix READ headMatrix NOTIFY headMatrixChanged)
    Q_PROPERTY(QMatrix4x4 headOrientation READ headOrientation NOTIFY headOrientationChanged)
    Q_PROPERTY(bool mirrorEnabled READ mirrorEnabled WRITE setMirrorEnabled NOTIFY mirrorEnabledChanged)
    Q_PROPERTY(bool mirrorDistorsion READ mirrorDistorsion WRITE setMirrorDistorsion NOTIFY mirrorDistorsionChanged)
    Q_PROPERTY(bool mirrorRightEye READ mirrorRightEye WRITE setMirrorRightEye NOTIFY mirrorRightEyeChanged)
    Q_PROPERTY(bool running READ running NOTIFY runningChanged)
    Q_PROPERTY(qreal pointSize READ pointSize WRITE setPointSize NOTIFY pointSizeChanged)
    Q_PROPERTY(qreal distanceDetail READ distanceDetail WRITE setDistanceDetail NOTIFY distanceDetailChanged)

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

    bool mirrorEnabled() const
    {
        return m_mirrorEnabled;
    }

    bool mirrorDistorsion() const
    {
        return m_mirrorDistorsion;
    }

    bool mirrorRightEye() const
    {
        return m_mirrorRightEye;
    }

    bool running() const
    {
        return m_running;
    }

    qreal pointSize() const
    {
        return m_pointSize;
    }

    QString filename() const
    {
        return m_filename;
    }

    qreal distanceDetail() const
    {
        return m_distanceDetail;
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

    void setMirrorEnabled(bool mirrorEnabled)
    {
        if (m_mirrorEnabled == mirrorEnabled)
            return;

        m_mirrorEnabled = mirrorEnabled;
        Q_EMIT mirrorEnabledChanged(mirrorEnabled);
    }

    void setMirrorDistorsion(bool mirrorDistorsion)
    {
        if (m_mirrorDistorsion == mirrorDistorsion)
            return;

        m_mirrorDistorsion = mirrorDistorsion;
        Q_EMIT mirrorDistorsionChanged(mirrorDistorsion);
    }

    void setMirrorRightEye(bool mirrorRightEye)
    {
        if (m_mirrorRightEye == mirrorRightEye)
            return;

        m_mirrorRightEye = mirrorRightEye;
        Q_EMIT mirrorRightEyeChanged(mirrorRightEye);
    }


    void setPointSize(qreal pointSize)
    {
        if (m_pointSize == pointSize)
            return;

        m_pointSize = pointSize;
        Q_EMIT pointSizeChanged(pointSize);
    }

    void setFilename(QString filename)
    {
        if (m_filename == filename)
            return;

        m_filename = filename;
        Q_EMIT filenameChanged(filename);
    }

    void setDistanceDetail(qreal distanceDetail)
    {
        if (m_distanceDetail == distanceDetail)
            return;

        m_distanceDetail = distanceDetail;
        Q_EMIT distanceDetailChanged(distanceDetail);
    }

    void emitFrame();

Q_SIGNALS:
    void frame();

    void matrixChanged(QMatrix4x4 matrix);

    void entitydataChanged(QmlEntitydata * entitydata);
    void updated(upns::upnsSharedPointer<upns::AbstractEntityData> entitydata);

    void needsReload();
    void vrmodeChanged(bool vrmode);

    void headDirectionChanged(QVector3D headDirection);
    void headMatrixChanged(QMatrix4x4 headMatrix);
    void headOrientationChanged(QMatrix4x4 headOrientation);

    void mirrorEnabledChanged(bool mirrorEnabled);

    void mirrorDistorsionChanged(bool mirrorDistorsion);

    void mirrorRightEyeChanged(bool mirrorRightEye);

    void runningChanged(bool running);

    void pointSizeChanged(qreal pointSize);

    void filenameChanged(QString filename);

    void distanceDetailChanged(qreal distanceDetail);

protected:
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);

private Q_SLOTS:
    void setHeadDirection(QVector3D headDirection);
    void setHeadOrientation(QMatrix4x4 headOrientation);
    void setHeadMatrix(QMatrix4x4 headMatrix);
    void setRunning(bool running);

private:
    RenderThread *m_renderThread;
    QMatrix4x4 m_matrix;
    QmlEntitydata * m_entitydata;
    upns::upnsSharedPointer<QMetaObject::Connection> m_connectionToEntityData;
    bool m_vrmode;
    QVector3D m_headDirection;
    QMatrix4x4 m_headMatrix;
    QMatrix4x4 m_headOrientation;
    bool m_mirrorEnabled;
    bool m_mirrorDistorsion;
    bool m_mirrorRightEye;
    bool m_running;
    qreal m_pointSize;
    QString m_filename;
    qreal m_distanceDetail;
};

#endif // MapsRenderer_H
