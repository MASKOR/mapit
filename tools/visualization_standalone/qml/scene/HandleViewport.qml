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
