#ifndef QMLRAYCAST_H
#define QMLRAYCAST_H

#include <QMatrix4x4>
#include <QVector2D>

///
/// \brief The QmlRaycast class helps projecting and unprojecting rays
/// It can be used to
/// 1) unproject a 2d point on screen to 3d point in world at a given distance
/// 2) unproject a 2d point on screen to 3d point in world at a given plane
/// 3) project 3d point from world to screen
/// 4) direction from camera to an point in world or on screen
/// This can be done in a declaraive way by setting the needed parameters
/// and retrieve whatever is needed. Computation is done lazyly,
/// however, property bindings are evaluated correctly and thus may
/// break lazy evaluation.
///
class QmlRaycast : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QMatrix4x4 viewMatrix READ viewMatrix WRITE setViewMatrix NOTIFY viewMatrixChanged)
    Q_PROPERTY(QMatrix4x4 projectionMatrix READ projectionMatrix WRITE setProjectionMatrix NOTIFY projectionMatrixChanged)
    Q_PROPERTY(QSize viewportSize READ viewportSize WRITE setViewportSize NOTIFY viewportSizeChanged)
    Q_PROPERTY(QVector2D screenPosition READ screenPosition WRITE setScreenPosition NOTIFY screenPositionChanged)
    Q_PROPERTY(QVector3D worldDirection READ worldDirection WRITE setWorldDirection NOTIFY worldDirectionChanged)
    Q_PROPERTY(QVector3D worldPosition READ worldPosition WRITE setWorldPosition NOTIFY worldPositionChanged)
    Q_PROPERTY(float distance READ distance WRITE setDistance NOTIFY distanceChanged)
    Q_PROPERTY(QVector3D planeNormal READ planeNormal WRITE setPlaneNormal NOTIFY planeNormalChanged)
    Q_PROPERTY(float planeOffset READ planeOffset WRITE setPlaneOffset NOTIFY planeOffsetChanged)
    Q_PROPERTY(QVector3D pointOnPlane READ pointOnPlane WRITE setPointOnPlane NOTIFY pointOnPlaneChanged)

public:
    QmlRaycast(QObject* parent = nullptr);

    QMatrix4x4 viewMatrix() const;
    QSize viewportSize() const;
    QVector2D screenPosition();
    QVector3D worldDirection();
    QVector3D worldPosition();
    float distance();
    QMatrix4x4 projectionMatrix() const;
    QVector3D planeNormal();
    float planeOffset();
    QVector3D pointOnPlane();

public Q_SLOTS:
    void setViewMatrix(QMatrix4x4 viewMatrix);
    void setViewportSize(QSize viewportSize);
    void setScreenPosition(QVector2D screenPosition);
    void setWorldDirection(QVector3D worldDirection);
    void setWorldPosition(QVector3D worldPosition);
    void setDistance(float distance);
    void setProjectionMatrix(QMatrix4x4 projectionMatrix);
    void setPlaneNormal(QVector3D planeNormal);
    void setPlaneOffset(float planeOffset);
    void setPointOnPlane(QVector3D pointOnPlane);

Q_SIGNALS:
    void viewMatrixChanged(QMatrix4x4 viewMatrix);
    void viewportSizeChanged(QSize viewportSize);
    void screenPositionChanged();
    void worldDirectionChanged();
    void worldPositionChanged();
    void distanceChanged();
    void projectionMatrixChanged(QMatrix4x4 projectionMatrix);
    void planeNormalChanged();
    void planeOffsetChanged();
    void pointOnPlaneChanged();

private:
    QMatrix4x4 m_viewMatrix;
    QSize m_viewportSize;
    QVector2D m_screenPosition;
    QVector3D m_worldDirection;
    QVector3D m_worldPosition;
    float m_distance;
    QMatrix4x4 m_projectionMatrix;
    QVector3D m_planeNormal;
    QVector3D m_pointOnPlane;
    float m_planeOffset;
    bool m_dirtyScreenPosition;
    bool m_dirtyWorldDirection;
    bool m_dirtyWorldPosition;
    bool m_dirtyDistance;
    bool m_dirtyPlaneNormal;
    bool m_dirtyPlaneOffset;
    bool m_dirtyPointOnPlane;
};

QT_END_NAMESPACE

#endif
