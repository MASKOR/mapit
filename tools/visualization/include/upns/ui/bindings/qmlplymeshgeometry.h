/*******************************************************************************
 *
 * Copyright 2017-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#ifndef QMLPLAYMESHGEOMETRY_H
#define QMLPLAYMESHGEOMETRY_H

#include <Qt3DRender/qgeometry.h>
#include <upns/layertypes/assettype.h>

class QmlPlyMeshGeometryPrivate;

class QmlPlyMeshGeometry : public Qt3DRender::QGeometry
{
    Q_OBJECT
public:
    explicit QmlPlyMeshGeometry(QNode *parent = NULL);
    ~QmlPlyMeshGeometry();
    void updateVertices();

public Q_SLOTS:
    void setAsset(AssetPtr asset);
private Q_SLOTS:
    void updateAttributes(uint32_t vertexCount, bool hasNormals);
Q_SIGNALS:
    void assetChanged(AssetPtr asset);
private:
    QmlPlyMeshGeometryPrivate *m_p;
};


#endif
