#include "bindings/renderdata.h"
#include <QQuickItem>
#include <QMetaObject>

Renderdata::Renderdata()
   :m_entitydata( nullptr ),
    m_width( 1 ),
    m_height( 1 ),
    m_matrix( ),
    m_vrmode( false ),
    m_mirrorEnabled( true ),
    m_mirrorDistorsion( true ),
    m_mirrorRightEye( false ),
    m_pointSize( 64.f ),
    m_distanceDetail( 1.f ),
    m_filename( "" ),
    m_headMatrix( ),
    m_headDirection( ),
    m_headOrientation( ),
    m_running( false ),
    m_connectionToEntityData( nullptr )
{
}

void Renderdata::connectReadInputWidthHeightFrom(Renderdata *other)
{
    connect(other, &Renderdata::widthChanged, this, &Renderdata::setWidth);
    connect(other, &Renderdata::heightChanged, this, &Renderdata::setHeight);
}

void Renderdata::connectReadInputWidthHeightFrom(QQuickItem *other)
{
    connect(other, &QQuickItem::widthChanged, this, [other, this]()
    {
        //if(this->vrmode())
            this->setWidth(other->width());
    });
    connect(other, &QQuickItem::heightChanged, this, [other, this]()
    {
        //if(this->vrmode())
            this->setHeight(other->height());
    });
}

void Renderdata::connectReadInputFrom(Renderdata *other)
{
    connect(other, &Renderdata::entitydataChanged, this, &Renderdata::setEntitydata);
    connect(other, &Renderdata::filenameChanged, this, &Renderdata::setFilename);
    //TODO: connect(other, &Renderdata::needsReload, this, &Renderdata::reload);
    connect(other, &Renderdata::matrixChanged, this, &Renderdata::setMatrix);
    connect(other, &Renderdata::vrmodeChanged, this, &Renderdata::setVrmode);
    connect(other, &Renderdata::mirrorEnabledChanged, this, &Renderdata::setMirrorEnabled);
    connect(other, &Renderdata::mirrorDistorsionChanged, this, &Renderdata::setMirrorDistorsion);
    connect(other, &Renderdata::mirrorRightEyeChanged, this, &Renderdata::setMirrorRightEye);
    connect(other, &Renderdata::pointSizeChanged, this, &Renderdata::setPointSize);
    connect(other, &Renderdata::distanceDetailChanged, this, &Renderdata::setDistanceDetail);
}

void Renderdata::connectWriteOutputTo(Renderdata *other)
{
    connect(this, &Renderdata::headDirectionChanged, other, &Renderdata::setHeadDirection);
    connect(this, &Renderdata::headMatrixChanged, other, &Renderdata::setHeadMatrix);
    connect(this, &Renderdata::headOrientationChanged, other, &Renderdata::setHeadOrientation);
    connect(this, &Renderdata::runningChanged, other, &Renderdata::setRunning);
}

QmlEntitydata *Renderdata::entitydata() const
{
    return m_entitydata;
}

qreal Renderdata::width() const
{
    return m_width;
}

qreal Renderdata::height() const
{
    return m_height;
}

QMatrix4x4 Renderdata::matrix() const
{
    return m_matrix;
}

bool Renderdata::vrmode() const
{
    return m_vrmode;
}

bool Renderdata::mirrorEnabled() const
{
    return m_mirrorEnabled;
}

bool Renderdata::mirrorDistorsion() const
{
    return m_mirrorDistorsion;
}

bool Renderdata::mirrorRightEye() const
{
    return m_mirrorRightEye;
}

qreal Renderdata::pointSize() const
{
    return m_pointSize;
}

qreal Renderdata::distanceDetail() const
{
    return m_distanceDetail;
}

QString Renderdata::filename() const
{
    return m_filename;
}

QMatrix4x4 Renderdata::headMatrix() const
{
    return m_headMatrix;
}

QVector3D Renderdata::headDirection() const
{
    return m_headDirection;
}

QMatrix4x4 Renderdata::headOrientation() const
{
    return m_headOrientation;
}

bool Renderdata::running() const
{
    return m_running;
}

void Renderdata::setEntitydata(QmlEntitydata *entitydata)
{
    if (m_entitydata == entitydata)
        return;
    // QmlEntitydata may change its inner, wrapped entitydata.
    // When this happens, a usual entityDataChanged signal is fired
    if(m_entitydata != NULL && m_connectionToEntityData != NULL)
    {
        disconnect(*m_connectionToEntityData);
        m_connectionToEntityData = NULL;
    }
    m_entitydata = entitydata;
    if(m_entitydata)
    {
        upns::upnsSharedPointer<QMetaObject::Connection> con( new QMetaObject::Connection(
                                                                  connect(m_entitydata,
                                                                          &QmlEntitydata::updated,
                                                                          this,
                                                                          [&](){this->emitEntitiydataChanged(m_entitydata);})));
        m_connectionToEntityData = con;
    }
    Q_EMIT entitydataChanged(entitydata);
}

void Renderdata::setWidth(qreal width)
{
    if (m_width == width)
        return;

    m_width = width;
    Q_EMIT widthChanged(width);
}

void Renderdata::setHeight(qreal height)
{
    if (m_height == height)
        return;

    m_height = height;
    Q_EMIT heightChanged(height);
}

void Renderdata::setMatrix(QMatrix4x4 matrix)
{
    if (m_matrix == matrix)
        return;

    m_matrix = matrix;
    Q_EMIT matrixChanged(matrix);
}

void Renderdata::setVrmode(bool vrmode)
{
    if (m_vrmode == vrmode)
        return;

    m_vrmode = vrmode;
    Q_EMIT vrmodeChanged(vrmode);
}

void Renderdata::setMirrorEnabled(bool mirrorEnabled)
{
    if (m_mirrorEnabled == mirrorEnabled)
        return;

    m_mirrorEnabled = mirrorEnabled;
    Q_EMIT mirrorEnabledChanged(mirrorEnabled);
}

void Renderdata::setMirrorDistorsion(bool mirrorDistorsion)
{
    if (m_mirrorDistorsion == mirrorDistorsion)
        return;

    m_mirrorDistorsion = mirrorDistorsion;
    Q_EMIT mirrorDistorsionChanged(mirrorDistorsion);
}

void Renderdata::setMirrorRightEye(bool mirrorRightEye)
{
    if (m_mirrorRightEye == mirrorRightEye)
        return;

    m_mirrorRightEye = mirrorRightEye;
    Q_EMIT mirrorRightEyeChanged(mirrorRightEye);
}

void Renderdata::setPointSize(qreal pointSize)
{
    if (m_pointSize == pointSize)
        return;

    m_pointSize = pointSize;
    Q_EMIT pointSizeChanged(pointSize);
}

void Renderdata::setDistanceDetail(qreal distanceDetail)
{
    if (m_distanceDetail == distanceDetail)
        return;

    m_distanceDetail = distanceDetail;
    Q_EMIT distanceDetailChanged(distanceDetail);
}

void Renderdata::setFilename(QString filename)
{
    if (m_filename == filename)
        return;

    m_filename = filename;
    Q_EMIT filenameChanged(filename);
}

void Renderdata::setHeadMatrix(QMatrix4x4 headMatrix)
{
    if (m_headMatrix == headMatrix)
        return;

    m_headMatrix = headMatrix;
    Q_EMIT headMatrixChanged(headMatrix);
}

void Renderdata::setHeadDirection(QVector3D headDirection)
{
    if (m_headDirection == headDirection)
        return;

    m_headDirection = headDirection;
    Q_EMIT headDirectionChanged(headDirection);
}

void Renderdata::setHeadOrientation(QMatrix4x4 headOrientation)
{
    if (m_headOrientation == headOrientation)
        return;

    m_headOrientation = headOrientation;
    Q_EMIT headOrientationChanged(headOrientation);
}

void Renderdata::setRunning(bool running)
{
    if (m_running == running)
        return;

    m_running = running;
    Q_EMIT runningChanged(running);
}

void Renderdata::emitEntitiydataChanged(QmlEntitydata *entitydata)
{
    Q_EMIT entitydataChanged(entitydata);
}


#include "renderdata.moc"
