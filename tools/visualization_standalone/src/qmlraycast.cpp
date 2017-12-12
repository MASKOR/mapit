 #include "qmlraycast.h"

QmlRaycast::QmlRaycast(QObject *parent)
    :QObject(parent)
    ,m_viewMatrix()
    ,m_viewportSize()
    ,m_screenPosition()
    ,m_worldDirection()
    ,m_worldPosition()
    ,m_distance()
    ,m_planeNormal()
    ,m_planeOffset(0.0f)
    ,m_pointOnPlane()
    ,m_dirtyScreenPosition(true)
    ,m_dirtyWorldDirection(true)
    ,m_dirtyWorldPosition(true)
    ,m_dirtyDistance(true)
    ,m_dirtyPlaneNormal(true)
    ,m_dirtyPlaneOffset(true)
    ,m_dirtyPointOnPlane(true)
{}

QMatrix4x4 QmlRaycast::viewMatrix() const
{
    return m_viewMatrix;
}

void QmlRaycast::setViewMatrix(QMatrix4x4 viewMatrix)
{
    if (m_viewMatrix == viewMatrix)
        return;

    m_viewMatrix = viewMatrix;
    Q_EMIT viewMatrixChanged(m_viewMatrix);
}

QSize QmlRaycast::viewportSize() const
{
    return m_viewportSize;
}

void QmlRaycast::setViewportSize(QSize viewportSize)
{
    if (m_viewportSize == viewportSize)
        return;

    m_viewportSize = viewportSize;
    Q_EMIT viewportSizeChanged(m_viewportSize);
}

QVector2D QmlRaycast::screenPosition()
{
    if(m_dirtyScreenPosition) {
        QVector3D worldPos(worldPosition());
        if(m_dirtyWorldPosition) {
            qWarning("screenPosition can not be determined. Please provide worldDirection or worldPosition.");
            return QVector2D(0.0f, 0.0f);
        }
        QMatrix4x4 viewProjectionMatrix = projectionMatrix() * viewMatrix();
        QVector3D clippingCoord(viewProjectionMatrix * QVector4D(worldPos, 1.0));
        float x = (( clippingCoord.x() + 1 ) / 2.0) * viewportSize().width();
        float y = (( 1 - clippingCoord.y() ) / 2.0) * viewportSize().height();
        m_screenPosition = QVector2D(x, y);
        m_dirtyScreenPosition = false;
    }
    return m_screenPosition;
}

void QmlRaycast::setScreenPosition(QVector2D screenPosition)
{
    m_dirtyScreenPosition = false;
    if (m_screenPosition == screenPosition)
        return;

    m_screenPosition = screenPosition;
    m_dirtyWorldDirection = true;
    m_dirtyWorldPosition = true;
    Q_EMIT screenPositionChanged();
    Q_EMIT worldDirectionChanged();
    Q_EMIT worldPositionChanged();
}

QVector3D QmlRaycast::worldDirection()
{
    if(m_dirtyWorldDirection) {
        if(!m_dirtyScreenPosition) {
            float x = ((2.0f * m_screenPosition.x()) / m_viewportSize.width()) - 1.0f;
            float y = 1.0f - ((2.0f * m_screenPosition.y()) / m_viewportSize.height());

            // Figure out the ray to the screen position
            QVector4D ray = projectionMatrix().inverted() * QVector4D(x, y, -1.0f, 1.0f);
            ray.setZ(-1.0f);
            ray.setW(0.0f);
            ray = viewMatrix().inverted() * ray;
            QVector3D worldDirectionNew = ray.toVector3D().normalized();
            m_worldDirection = worldDirectionNew;
            m_dirtyWorldDirection = false;
        } else if(!m_dirtyWorldPosition) {
            float *data = viewMatrix().inverted().data();
            QVector3D cameraPosition(data[12], data[13], data[14]);
            QVector3D worldDirectionNew = (worldPosition()-cameraPosition).normalized();
            m_worldDirection = worldDirectionNew;
            m_dirtyWorldDirection = false;
        }
    }
    return m_worldDirection;
}

void QmlRaycast::setWorldDirection(QVector3D worldDirection)
{
    m_dirtyWorldDirection = false;
    if (m_worldDirection == worldDirection)
        return;

    m_worldDirection = worldDirection;
    m_dirtyWorldPosition = true;
    m_dirtyScreenPosition = true;
    Q_EMIT worldDirectionChanged();
    Q_EMIT worldPositionChanged();
    Q_EMIT screenPositionChanged();
}

QVector3D QmlRaycast::worldPosition()
{
    // may triggers calculating worldDirection and thus screenPosition.
    if(m_dirtyWorldPosition) {
        //distance must be known, can not be retrieved/calculated
        if(m_dirtyDistance == (m_dirtyPlaneNormal && m_dirtyPlaneOffset && m_dirtyPointOnPlane)) {
            if(m_dirtyDistance)
                qWarning("worldPosition can not be determined. Please provide distance, worldPosition xor plane.");
            else
                qWarning("worldPosition can not be determined. Please provide only distance xor plane.");
            return QVector3D(0.0f, 0.0f, 0.0f);
        }
        if(!m_dirtyDistance) {
            float *data = viewMatrix().inverted().data();
            QVector3D cameraPosition(data[12], data[13], data[14]);
            QVector3D worldDir(worldDirection());
            if(m_dirtyWorldDirection) {
                qWarning("worldPosition can not be determined. Please provide worldDirection or screenPosition.");
                return QVector3D(0.0f, 0.0f, 0.0f);
            }
            QVector3D planeNormal(worldDir);
            float divisor = QVector3D::dotProduct(worldDir, planeNormal);
            if (qFuzzyCompare(1.0f, 1.0f + divisor)) {
                // plane perpendicular to ray
                m_distance = 0.0;
                m_worldPosition = cameraPosition;
                m_dirtyWorldPosition = false;
            } else {
                float t = -(QVector3D::dotProduct(cameraPosition, planeNormal) - m_distance) / divisor;
                m_worldPosition = cameraPosition + worldDir * t;
                m_dirtyWorldPosition = false;
            }
        } else if(!m_dirtyPlaneNormal && !m_dirtyPlaneOffset) {
            float *data = viewMatrix().inverted().data();
            QVector3D cameraPosition(data[12], data[13], data[14]);
            QVector3D worldDir(worldDirection());
            if(m_dirtyWorldDirection) {
                qWarning("worldPosition can not be determined. Please provide worldDirection or screenPosition.");
                return QVector3D(0.0f, 0.0f, 0.0f);
            }
            float divisor = QVector3D::dotProduct(worldDir, m_planeNormal);
            if (qFuzzyCompare(1.0f, 1.0f + divisor)) {
                // plane perpendicular to ray
                m_distance = 0.0;
                m_worldPosition = cameraPosition;
                m_dirtyWorldPosition = false;
            } else {
                float t = -(QVector3D::dotProduct(cameraPosition, m_planeNormal) - planeOffset()) / divisor;
                m_worldPosition = cameraPosition + worldDir * t;
                m_dirtyWorldPosition = false;
            }
        } else if(!m_dirtyPlaneNormal && !m_dirtyPointOnPlane) {
            float *data = viewMatrix().inverted().data();
            QVector3D cameraPosition(data[12], data[13], data[14]);
            QVector3D worldDir(worldDirection());
            if(m_dirtyWorldDirection) {
                qWarning("worldPosition can not be determined. Please provide worldDirection or screenPosition.");
                return QVector3D(0.0f, 0.0f, 0.0f);
            }
            float divisor = QVector3D::dotProduct(worldDir, m_planeNormal);
            if (qFuzzyCompare(1.0f, 1.0f + divisor)) {
                // plane perpendicular to ray
                m_distance = 0.0;
                m_worldPosition = cameraPosition;
                m_dirtyWorldPosition = false;
            } else {
                // plane offset is later calculated by new
                float cosinus = QVector3D::dotProduct(pointOnPlane().normalized(), planeNormal());
                float planeOffset = m_pointOnPlane.length() * cosinus;
                float t = -(QVector3D::dotProduct(cameraPosition, m_planeNormal) - planeOffset) / divisor;
                m_worldPosition = cameraPosition + worldDir * t;
                m_dirtyWorldPosition = false;
            }
        }
    }
    return m_worldPosition;
}

void QmlRaycast::setWorldPosition(QVector3D worldPosition)
{
    m_dirtyWorldPosition = false;
    if (m_worldPosition == worldPosition)
        return;

    m_worldPosition = worldPosition;
    m_dirtyScreenPosition = true;
    m_dirtyWorldDirection = true;
    m_dirtyDistance = true;
    m_dirtyPlaneOffset = true;
    m_dirtyPointOnPlane = true;
    Q_EMIT worldPositionChanged();
    Q_EMIT screenPositionChanged();
    Q_EMIT worldDirectionChanged();
    Q_EMIT distanceChanged();
    Q_EMIT planeOffsetChanged();
    Q_EMIT pointOnPlaneChanged();
}

float QmlRaycast::distance()
{
    if(m_dirtyDistance) {
        // depends on worldPosition
        float *data = viewMatrix().inverted().data();
        QVector3D cameraPositon(data[12], data[13], data[14]);
        QVector3D worldPos(worldPosition());
        if(m_dirtyWorldPosition) {
            qWarning("distance can not be determined. Please provide worldPosition.");
            return 0.0f;
        }
        m_distance = worldPos.distanceToPoint(cameraPositon);
        m_dirtyDistance = false;
    }
    return m_distance;
}

void QmlRaycast::setDistance(float distance)
{
    m_dirtyDistance = false;
    qWarning("Floating point comparison needs context sanity check");
    if (qFuzzyCompare(m_distance, distance))
        return;

    m_distance = distance;
    m_dirtyWorldPosition = true;
    m_dirtyPlaneOffset = true;
    m_dirtyPointOnPlane = true;
    Q_EMIT distanceChanged();
    Q_EMIT worldPositionChanged();
    Q_EMIT planeOffsetChanged();
    Q_EMIT pointOnPlaneChanged();
}

QMatrix4x4 QmlRaycast::projectionMatrix() const
{
    return m_projectionMatrix;
}

QVector3D QmlRaycast::planeNormal()
{
    return m_planeNormal;
}

float QmlRaycast::planeOffset()
{
    if(m_dirtyPlaneOffset) {
        if(!m_dirtyPointOnPlane) {
            float cosinus = QVector3D::dotProduct(pointOnPlane().normalized(), planeNormal());
            m_planeOffset = m_pointOnPlane.length() * cosinus;
            m_dirtyPlaneOffset = false;
        } else {
            float cosinus = QVector3D::dotProduct(worldPosition().normalized(), planeNormal());
            m_planeOffset = m_worldPosition.length() * cosinus;
            m_dirtyPlaneOffset = false;
        }
    }
    return m_planeOffset;
}

QVector3D QmlRaycast::pointOnPlane()
{
    if(m_dirtyPointOnPlane) {
        if(m_dirtyPlaneOffset) {
            qWarning("pointOnPlane can not be calculated. Pleasy provide pointOnPlane or planeOffset.");
            return QVector3D(0.0f, 0.0f, 0.0f);
        }
        if(!m_dirtyPlaneOffset) {
            m_pointOnPlane = planeNormal() * m_planeOffset;
            m_dirtyPointOnPlane = true;
        }
    }
    return m_pointOnPlane;
}

void QmlRaycast::setProjectionMatrix(QMatrix4x4 projectionMatrix)
{
    if (m_projectionMatrix == projectionMatrix)
        return;

    m_projectionMatrix = projectionMatrix;
    Q_EMIT projectionMatrixChanged(m_projectionMatrix);
}

void QmlRaycast::setPlaneNormal(QVector3D planeNormal)
{
    m_dirtyPlaneNormal = false;
    if (m_planeNormal == planeNormal)
        return;

    m_planeNormal = planeNormal.normalized();
    m_dirtyWorldPosition = true;
    m_dirtyDistance = true;
    m_dirtyPlaneOffset = true;
    Q_EMIT planeNormalChanged();
    Q_EMIT worldPositionChanged();
    Q_EMIT distanceChanged();
    Q_EMIT planeOffsetChanged();
}

void QmlRaycast::setPlaneOffset(float planeOffset)
{
    m_dirtyPlaneOffset = false;
    if (qFuzzyCompare(m_planeOffset, planeOffset))
        return;

    m_planeOffset = planeOffset;
    m_dirtyWorldPosition = true;
    m_dirtyPointOnPlane = true;
    Q_EMIT planeOffsetChanged();
    Q_EMIT worldPositionChanged();
    Q_EMIT pointOnPlaneChanged();
}

void QmlRaycast::setPointOnPlane(QVector3D pointOnPlane)
{
    m_dirtyPointOnPlane = false;
    if (m_pointOnPlane == pointOnPlane)
        return;

    m_pointOnPlane = pointOnPlane;
    m_dirtyPlaneOffset = true;
    m_dirtyWorldPosition = true;
    m_dirtyDistance = true;
    Q_EMIT pointOnPlaneChanged();
    Q_EMIT planeOffsetChanged();
    Q_EMIT worldPositionChanged();
    Q_EMIT distanceChanged();
}
