import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1
import QtGraphicalEffects 1.0
import QtQuick.Scene3D 2.0
import Qt3D.Core 2.0 as Q3D
import Qt3D.Render 2.0
import Qt3D.Input 2.0
//import Qt3D.Extras 2.0

import fhac.upns 1.0 as UPNS
//import "components"
//import "operators"

import QtQuick 2.0 as QQ2

ApplicationWindow {
    title: qsTr("Map Visualization")
    width: 1200
    height: 800
    visible: true
    menuBar: MainMenubar {
        id: menubar
        //uiEnabled: drawingArea.renderdata.running
    }
    UPNS.Repository {
        id: repo
        conf: "./repo.yaml"
    }
    RowLayout {
        anchors.fill: parent
        ColumnLayout {
            Text { text: "PointSize: " + pointSizeSlider.value.toFixed(2) }
            Slider {
                id: pointSizeSlider
                width: 100
                value: 0.5
                minimumValue: 0.01
                maximumValue:  2.0
            }
        }
        Layout.fillWidth: true

        Scene3D {
            id: scene3d
            Layout.minimumWidth: 50
            Layout.fillWidth: true
            Layout.fillHeight: true
            aspects: ["render", "logic", "input"]
            focus: true

            Q3D.Entity {
                id: sceneRoot
                Q3D.Camera {
                    id: camera
                    projectionType: Q3D.CameraLens.PerspectiveProjection
                    fieldOfView: 45
                    aspectRatio: scene3d.width/scene3d.height
                    nearPlane : 0.1
                    farPlane : 1000.0
                    position: Qt.vector3d( 0.0, 0.0, -40.0 )
                    upVector: Qt.vector3d( 0.0, 1.0, 0.0 )
                    viewCenter: Qt.vector3d( 0.0, 0.0, 0.0 )
                }

                Q3D.Configuration  {
                    controlledCamera: camera
                }

                components: [
                    FrameGraph {
                        activeFrameGraph: Viewport {
                            id: viewport
                            rect: Qt.rect(0.0, 0.0, 1.0, 1.0) // From Top Left
                            clearColor: "transparent"

                            CameraSelector {
                                id : cameraSelector
                                camera: camera
                                ClearBuffer {
                                    buffers : ClearBuffer.ColorDepthBuffer
                                    LayerFilter {
                                        layers: ["points"]
                                        StateSet {
                                            renderStates: [
                                                //PointSize { specification: PointSize.StaticValue; value: 5 },
                                                PointSize { specification: PointSize.Programmable },
                                                DepthTest { func: DepthTest.Less },
                                                DepthMask { mask: true }
                                            ]
                                        }
                                    }
                                }
                                LayerFilter {
                                    layers: ["solid"]
                                }
                            }
                        }
                    }
                ]

                Q3D.Entity {
                    id: pointcloud
                    property Layer layerPoints: Layer {
                            names: "points"
                        }
                    property var meshTransform: Q3D.Transform {
                            property real userAngle: -90.0
                            scale: 10
                            rotation: fromAxisAndAngle(Qt.vector3d(1, 0, 0), userAngle)
                        }
                    property GeometryRenderer customMesh: UPNS.EntitydataRenderer {
                            entitydata: repo.getCheckout("testcheckout").getEntitydataReadOnly("corridor/laser/eins")
                        }
                    property Material materialPoint: Material {
                        effect: Effect {
                            techniques: Technique {
                                renderPasses: RenderPass {
                                    shaderProgram: ShaderProgram {
                                        //vertexShaderCode: loadSource("qrc:/shader/pointcloud.vert")
                                        //fragmentShaderCode: loadSource("qrc:/shader/pointcloud.frag")
                                        vertexShaderCode: loadSource("qrc:/shader/surfel.vert")
                                        fragmentShaderCode: loadSource("qrc:/shader/surfel.frag")
                                    }
                                }
                            }
                        }
                        parameters: [
                            Parameter { name: "pointSize"; value: pointSizeSlider.value },
                            Parameter { name: "fieldOfView"; value: camera.fieldOfView },
                            Parameter { name: "fieldOfViewVertical"; value: camera.fieldOfView/camera.aspectRatio },
                            Parameter { name: "nearPlane"; value: camera.nearPlane },
                            Parameter { name: "farPlane"; value: camera.farPlane },
                            Parameter { name: "width"; value: scene3d.width },
                            Parameter { name: "height"; value: scene3d.height }
                        ]
                    }
                    components: [ customMesh, materialPoint, meshTransform, layerPoints ]
                }

                Q3D.Entity {
                    id: customEntity
                    property Layer layerPoints: Layer {
                            names: "noneZ"
                        }
                    property var meshTransform: Q3D.Transform {
                            property real userAngle: 0.0
                            scale: 10
                            rotation: fromAxisAndAngle(Qt.vector3d(0, 1, 0), userAngle)
                        }
                    property GeometryRenderer customMesh: GeometryRenderer {
                            instanceCount: 1
                            baseVertex: 0
                            baseInstance: 0
                            primitiveType: GeometryRenderer.Points
                            Buffer {
                                id: vertexBuffer
                                type: Buffer.VertexBuffer
                                data: {
                                        // Vertices
                                        var v0 = Qt.vector3d(-1.0, 0.0, -1.0)
                                        var v1 = Qt.vector3d(1.0, 0.0, -1.0)
                                        var v2 = Qt.vector3d(0.0, 1.0, 0.0)
                                        var v3 = Qt.vector3d(0.0, 0.0, 1.0)

                                        // Face Normals
                                        function normal(v0, v1, v2) {
                                            return v1.minus(v0).crossProduct(v2.minus(v0)).normalized();
                                        }
                                        var n023 = normal(v0, v2, v3)
                                        var n012 = normal(v0, v1, v2)
                                        var n310 = normal(v3, v1, v0)
                                        var n132 = normal(v1, v3, v2)

                                        // Vector normals
                                        var n0 = n023.plus(n012).plus(n310).normalized()
                                        var n1 = n132.plus(n012).plus(n310).normalized()
                                        var n2 = n132.plus(n012).plus(n023).normalized()
                                        var n3 = n132.plus(n310).plus(n023).normalized()

                                        // Colors
                                        var red = Qt.vector3d(1.0, 0.0, 0.0)
                                        var green = Qt.vector3d(0.0, 1.0, 0.0)
                                        var blue = Qt.vector3d(0.0, 0.0, 1.0)
                                        var white = Qt.vector3d(1.0, 1.0, 1.0)

                                        var vertices = [
                                                    v0, n0, red,
                                                    v1, n1, blue,
                                                    v2, n2, green,
                                                    v3, n3, white
                                                ]

                                        var vertexArray = new Float32Array(4 * (3 + 3 + 3));
                                        var i = 0;

                                        vertices.forEach(function(vec3) {
                                            vertexArray[i++] = vec3.x;
                                            vertexArray[i++] = vec3.y;
                                            vertexArray[i++] = vec3.z;
                                        });

                                        return vertexArray;
                                    }
                            }

                            geometry:  Geometry {
                                Attribute {
                                    attributeType: Attribute.VertexAttribute
                                    dataType: Attribute.Float
                                    dataSize: 3
                                    byteOffset: 0
                                    byteStride: 9 * 4
                                    count: 4
                                    name: defaultPositionAttributeName()
                                    buffer: vertexBuffer
                                }

                                Attribute {
                                    attributeType: Attribute.VertexAttribute
                                    dataType: Attribute.Float
                                    dataSize: 3
                                    byteOffset: 3 * 4
                                    byteStride: 9 * 4
                                    count: 4
                                    name: defaultNormalAttributeName()
                                    buffer: vertexBuffer
                                }

                                Attribute {
                                    attributeType: Attribute.VertexAttribute
                                    dataType: Attribute.Float
                                    dataSize: 3
                                    byteOffset: 6 * 4
                                    byteStride: 9 * 4
                                    count: 4
                                    name: defaultColorAttributeName()
                                    buffer: vertexBuffer
                                }
                            }
                        }
                    property Material materialPhong: PhongMaterial { }
                    components: [ customMesh, materialPhong, meshTransform, layerPoints ]
                }


                Q3D.Entity {
                    id: sphereEntity
                    property Layer layerSolid: Layer {
                            names: "solid"
                        }
                    property var meshTransform: Q3D.Transform {
                        id: sphereTransform
                        property real userAngle: 0.0
                        translation: Qt.vector3d(20, 0, 0)
                        rotation: fromAxisAndAngle(Qt.vector3d(0, 1, 0), userAngle)
                    }
                    property var sphereMesh: SphereMesh {
                        radius: 4
                    }
                    property Material materialPhong: PhongMaterial { }
                    components: [ sphereMesh, materialPhong, meshTransform, layerSolid ]
                }
                NumberAnimation {
                    target: sphereTransform
                    property: "userAngle"
                    duration: 10000
                    from: 0
                    to: 360

                    loops: Animation.Infinite
                    running: true
                }
            }
        }
    }

    SystemPalette {
        id: palette
    }
}
