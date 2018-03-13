/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#ifndef QMLENTITYDATAGEOMETRY_H
#define QMLENTITYDATAGEOMETRY_H

#include <Qt3DRender/QGeometryRenderer>
#include "qmlentitydata.h"
#include "qmlpointcloudcoordinatesystem.h"

class QmlEntitydataRenderer : public Qt3DRender::QGeometryRenderer
{
    Q_OBJECT
    Q_PROPERTY(QmlEntitydata* entitydata READ entitydata WRITE setEntitydata NOTIFY entitydataChanged)
    Q_PROPERTY(QmlPointcloudCoordinatesystem *coordinateSystem READ coordinateSystem WRITE setCoordinateSystem NOTIFY coordinateSystemChanged)

public:
    explicit QmlEntitydataRenderer(Qt3DCore::QNode *parent = Q_NULLPTR);
    QmlEntitydata* entitydata() const;

    Q_INVOKABLE void updateGeometry();

    QmlPointcloudCoordinatesystem * coordinateSystem() const;

public Q_SLOTS:
    void setEntitydata(QmlEntitydata* entitydata);

    void setCoordinateSystem(QmlPointcloudCoordinatesystem * coordinateSystem);

Q_SIGNALS:
    void entitydataChanged(QmlEntitydata* entitydata);

    void coordinateSystemChanged(QmlPointcloudCoordinatesystem * coordinateSystem);

private:
    QmlEntitydata* m_entitydata;

    void setInstanceCount(int instanceCount);
    void setPrimitiveCount(int primitiveCount);
    void setBaseVertex(int baseVertex);
    void setBaseInstance(int baseInstance);
    void setRestartIndex(int index);
    void setPrimitiveRestart(bool enabled);
    void setGeometry(Qt3DRender::QGeometry *geometry);
    void setPrimitiveType(PrimitiveType primitiveType);
    QmlPointcloudCoordinatesystem * m_coordinateSystem;
};

#endif
