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
    property real horizontalFov: 90
    property real aspectRatio: 16.0/9.0
    onHorizontalFovChanged: renderer.rebuild()
    onAspectRatioChanged: renderer.rebuild()
    property Layer layer
    property alias geometryRenderer: renderer
    Q3D.Entity {
        property Material material: PhongMaterial { diffuse: "red" }
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
                    count: 24 // 8 rays, 8 inner frame, 8 outer frame
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
                            var horizontalFovRad = root.horizontalFov * (Math.PI / 180.0)
                            var verticalFovRad = Math.atan( Math.tan( horizontalFovRad * 0.5 ) / root.aspectRatio ) * 2.0;
                            var horizM = Math.tan( horizontalFovRad * 0.5 )
                            var vertM = Math.tan( verticalFovRad * 0.5 )
                            var minLen = -1.0
                            var maxLen = -2.0
                            var origin = [0.0,0.0,0.0]
                            var innerFrame = [[-horizM * minLen, -vertM * minLen,  minLen]
                                             ,[-horizM * minLen,  vertM * minLen,  minLen]
                                             ,[ horizM * minLen,  vertM * minLen,  minLen]
                                             ,[ horizM * minLen, -vertM * minLen,  minLen]]
                            var outerFrame = [[-horizM * maxLen, -vertM * maxLen,  maxLen]
                                             ,[-horizM * maxLen,  vertM * maxLen,  maxLen]
                                             ,[ horizM * maxLen,  vertM * maxLen,  maxLen]
                                             ,[ horizM * maxLen, -vertM * maxLen,  maxLen]]
                            // inner frame
                            addLine(innerFrame[0], innerFrame[1]);
                            addLine(innerFrame[1], innerFrame[2]);
                            addLine(innerFrame[2], innerFrame[3]);
                            addLine(innerFrame[3], innerFrame[0]);

                            // outer frame
                            addLine(outerFrame[0], outerFrame[1]);
                            addLine(outerFrame[1], outerFrame[2]);
                            addLine(outerFrame[2], outerFrame[3]);
                            addLine(outerFrame[3], outerFrame[0]);

                            // rays
                            addLine(origin, outerFrame[0]);
                            addLine(origin, outerFrame[1]);
                            addLine(origin, outerFrame[2]);
                            addLine(origin, outerFrame[3]);

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
