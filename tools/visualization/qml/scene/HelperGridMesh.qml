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

import Qt3D.Core 2.0 as Q3D
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0

GeometryRenderer {
    id: root
    property real gridSpacing: 1
    property int lines: 20

    function rebuild() {
        buffer.data = buffer.buildGrid()
    }
    onGridSpacingChanged: rebuild()
    onLinesChanged: rebuild()
    instanceCount: 1
    indexOffset: 0
    firstInstance: 0
    vertexCount: root.lines*4
    primitiveType: GeometryRenderer.Lines
    geometry: Geometry {
        Attribute {
            id: positionAttribute
            attributeType: Attribute.VertexAttribute
            vertexBaseType: Attribute.Float
            vertexSize: 3
            byteOffset: 0
            byteStride: 3 * 4
            count: root.lines*4
            name: "vertexPosition"//defaultPositionAttributeName()
            buffer: Buffer {
                id: buffer
                type: Buffer.VertexBuffer
                function buildGrid() {
                    var vertices = root.lines*4;
                    var vertexFloats = 3;
                    var vertexArray = new Float32Array(vertexFloats * vertices);
                    for(var i=0 ; i<vertices ; i++) {
                        var line = Math.floor(i/2)%root.lines
                        var end = (i%2)?(-1):1
                        var d1 = ((-root.lines+1)*0.5+line)*gridSpacing
                        var d2 = end*(root.lines-1)*0.5*gridSpacing
                        if(i<(vertices*0.5)) {
                            vertexArray[i*vertexFloats]   = d1
                            vertexArray[i*vertexFloats+2] = d2
                        } else {
                            vertexArray[i*vertexFloats]   = d2
                            vertexArray[i*vertexFloats+2] = d1
                        }
                        vertexArray[i*vertexFloats+1] = 0
                    }
                    return vertexArray
                }
                data: buildGrid()
            }
        }
    }
}
