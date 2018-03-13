/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

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
