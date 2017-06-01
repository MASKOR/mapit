#ifndef QMLPOINTCLOUDCOORDINATESYSTEM_H
#define QMLPOINTCLOUDCOORDINATESYSTEM_H

#include <QObject>
///
/// \brief The QmlPointcloudCoordinatesystem class
/// Why?
/// LAS Files contain georeferenced datasets. In order to render them,
/// they must be brought into the same floatingpoint coordinate system.
/// How?
/// While loading, the double precision dataset is converted into float.
/// When loading the first dataset, a global offset is calculated.
/// The global offset is represented by this class.
/// Because qml can not handle doubles, this coordinate system is
/// invisble to the script.
/// When loading other pointclouds, this global origin should be used
/// instead of calculating a new one.
/// Other data?
/// Non georeferenced data is untouched by this. If e.g. a surface is
/// reconstructed from a georeferenced pointcloud, the resulting mesh
/// can be brought the geocoordinates using this global pointcloud
/// coordinate system.
/// But!
/// ... This class is not stored (yet) and is only used for visualization.
///
class QmlPointcloudCoordinatesystem : public QObject
{
    Q_OBJECT
public:
    bool m_initialized;
    double m_offsetX;
    double m_offsetY;
    double m_offsetZ;
    QmlPointcloudCoordinatesystem(QObject *parent = nullptr)
        : QObject(parent)
        , m_initialized(false)
        , m_offsetX(0.)
        , m_offsetY(0.)
        , m_offsetZ(0.)
    {}
};

#endif
