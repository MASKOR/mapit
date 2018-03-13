/*******************************************************************************
 *
 * Copyright 2016-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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

import QtQuick 2.4

import Qt3D.Core 2.0 as Q3D
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0

Q3D.Entity {
    property var currentEntitydataTransform
    id: handle
    property Layer layer: Layer {
            id: gizmoLayer
        }
    property var meshTransform: Q3D.Transform {
            matrix: currentEntitydataTransform.matrix
        }
    property GeometryRenderer gizmoMesh: TODO {

        }
    property Material gizmoMaterial: PhongMaterial { }
    components: [ gizmoMesh, gizmoMaterial, meshTransform, layer ]
}
