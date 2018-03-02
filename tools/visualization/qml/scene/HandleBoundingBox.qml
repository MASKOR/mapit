/*******************************************************************************
 *
 * Copyright      2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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

Q3D.Entity {
    id: root
    property vector3d min
    property vector3d max
    onMinChanged: renderer.rebuild()
    onMaxChanged: renderer.rebuild()
    property Layer layer
    Q3D.Entity {
        property Material material: PhongMaterial { ambient: "yellow" }
        property var trans: Q3D.Transform {
        }
        components: [ renderer, root.layer, material, trans ]
        GeometryRenderer {
            id: renderer

            function rebuild() {
                buffer.data = buffer.buildObj()
            }
            instanceCount: 1
            indexOffset: 0
            firstInstance: 0
            vertexCount: 24
            primitiveType: GeometryRenderer.Lines
            geometry: Geometry {
                Attribute {
                    id: positionAttribute
                    attributeType: Attribute.VertexAttribute
                    vertexBaseType: Attribute.Float
                    vertexSize: 3
                    byteOffset: 0
                    byteStride: 3 * 4
                    // top: 4 lines -> 8 verts, bottom: 8 verts, connectionTop/Bottom: 8 verts = 24
                    // (no index buffer used (yet), bounding boxes are not performance critical)
                    count: 24
                    name: "vertexPosition"//defaultPositionAttributeName()
                    buffer: Buffer {
                        id: buffer
                        type: Buffer.VertexBuffer
                        function buildObj() {
                            var vertices = 24
                            var vertexFloats = 3
                            var vertexArray = new Float32Array(vertexFloats * vertices)
                            var vertexCount = 0
                            var addLine = function(vert1, vert2) {
                                vertexArray[vertexCount*3  ] = vert1[0]
                                vertexArray[vertexCount*3+1] = vert1[1]
                                vertexArray[vertexCount*3+2] = vert1[2]
                                vertexArray[vertexCount*3+3] = vert2[0]
                                vertexArray[vertexCount*3+4] = vert2[1]
                                vertexArray[vertexCount*3+5] = vert2[2]
                                vertexCount += 2
                            }
                            var min = root.min
                            var max = root.max
                            var upper = [[min.x, max.y, min.z]
                                        ,[min.x, max.y, max.z]
                                        ,[max.x, max.y, max.z]
                                        ,[max.x, max.y, min.z]]
                            var lower = [[min.x, min.y, min.z]
                                        ,[min.x, min.y, max.z]
                                        ,[max.x, min.y, max.z]
                                        ,[max.x, min.y, min.z]]
                            // upper
                            addLine(upper[0], upper[1]);
                            addLine(upper[1], upper[2]);
                            addLine(upper[2], upper[3]);
                            addLine(upper[3], upper[0]);

                            // lower
                            addLine(lower[0], lower[1]);
                            addLine(lower[1], lower[2]);
                            addLine(lower[2], lower[3]);
                            addLine(lower[3], lower[0]);

                            // connections upper/lower
                            addLine(upper[0], lower[0]);
                            addLine(upper[1], lower[1]);
                            addLine(upper[2], lower[2]);
                            addLine(upper[3], lower[3]);

                            if(vertexCount !== positionAttribute.count) {
                                console.log("WARN: vertex count is wrong for frustum object")
                            }

                            return vertexArray
                        }
                        data: buildObj()
                    }
                }
            }
        }
    }
}
