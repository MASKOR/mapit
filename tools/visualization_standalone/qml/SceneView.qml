import QtQuick 2.4
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtQuick.Scene3D 2.0
import Qt3D.Core 2.0 as Q3D
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0

import pcl 1.0

import fhac.upns 1.0 as UPNS

import QtQuick 2.0 as QQ2

Item {
    id: root
    property real pointSize
    property var currentEntitydata
    property var currentEntitydataTranform
    property string currentEntityPath
    ColumnLayout {
        anchors.fill: parent
        //QCtl.ToolBar {
            RowLayout {
                id: toolbar
                Layout.fillWidth: true
                Layout.maximumHeight: 32
                AxisGizmo {
                    Layout.preferredHeight: toolbar.height
                    Layout.preferredWidth: toolbar.height
                    finalTransform: mainCamera.viewMatrix
                }
                Text { text: "PointSize: " + pointSizeSlider.value.toFixed(2); color: palette.text }
                QCtl.Slider {
                    id: pointSizeSlider
                    width: 100
                    value: 0.5
                    minimumValue: 0.01
                    maximumValue:  2.0
                }
            }
        //}

        MouseArea {
            Layout.fillWidth: true
            Layout.fillHeight: true
            id: sceneMouseArea


            hoverEnabled: true
            onEntered: priv.mouseOver = true
            onExited: {
                priv.mouseOver = false
            }

            Item {
                id: priv
                property bool mouseOver: true
            }

            Scene3D {
                anchors.fill: parent
                id: scene3d
                aspects: ["render", "logic", "input"]
                focus: priv.mouseOver
                Q3D.Entity {
                    id: sceneRoot
                    Camera {
                        id: mainCamera
                        projectionType: CameraLens.PerspectiveProjection
                        fieldOfView: 45
                        aspectRatio: scene3d.width/scene3d.height
                        nearPlane : 0.1
                        farPlane : 1000.0
                        position: Qt.vector3d( 0.0, 0.0, -40.0 )
                        upVector: Qt.vector3d( 0.0, 1.0, 0.0 )
                        viewCenter: Qt.vector3d( 0.0, 0.0, 0.0 )
                    }

                    FirstPersonCameraController {
                    //OrbitCameraController {
                        id: cameraController
                        camera: mainCamera
                        linearSpeed: priv.mouseOver*12.0
                    }

                    components: [
                        RenderSettings {
                            activeFrameGraph: Viewport {
                                id: viewport
                                normalizedRect: Qt.rect(0.0, 0.0, 1.0, 1.0) // From Top Left
                                RenderSurfaceSelector {
                                    CameraSelector {
                                        id : cameraSelector
                                        camera: mainCamera
                                        FrustumCulling {
                                            ClearBuffers {
                                                buffers : ClearBuffers.ColorDepthBuffer
                                                clearColor: "white"
                                                NoDraw {}
                                            }
                                            LayerFilter {
                                                layers: solidLayer
                                            }
                                            LayerFilter {
                                                layers: pointLayer
                                                RenderStateSet {
                                                    renderStates: [
                                                        //PointSize { sizeMode: PointSize.Fixed; value: 5.0 }, // exception when closing application in qt 5.7
                                                        PointSize { sizeMode: PointSize.Programmable }, //supported since OpenGL 3.2
                                                        DepthTest { depthFunction: DepthTest.Less }
                                                        //DepthMask { mask: true }
                                                    ]
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        },
                        // Event Source will be set by the Qt3DQuickWindow
                        InputSettings {
                            eventSource: scene3d
                            enabled: true
                        }
                    ]

                    Q3D.Entity {
                        id: pointcloud
                        property Layer layerPoints: Layer {
                                id: pointLayer
                            }
                        property ObjectPicker picker: ObjectPicker {
                            onClicked: console.log("Clicked pcd", pick.distance, pick.triangleIndex)
                        }
                        property var meshTransform: Q3D.Transform {
                                id: theMeshTransform
                                //property real userAngle: -90.0
                                //scale: 10
                                //rotation: fromAxisAndAngle(Qt.vector3d(1, 0, 0), userAngle)
                                matrix: root.currentEntitydataTransform ? root.currentEntitydataTransform.matrix : Qt.matrix4x4(1, 0, 0, 0,
                                                                                                                                0, 1, 0, 0,
                                                                                                                                0, 0, 1, 0,
                                                                                                                                0, 0, 0, 1)
                            }
                        property GeometryRenderer customMesh: UPNS.EntitydataRenderer {
                                entitydata: root.currentEntitydata
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
                                Parameter { name: "pointSize"; value: 5},//pointSizeSlider.value },
                                Parameter { name: "fieldOfView"; value: mainCamera.fieldOfView },
                                Parameter { name: "fieldOfViewVertical"; value: mainCamera.fieldOfView/mainCamera.aspectRatio },
                                Parameter { name: "nearPlane"; value: mainCamera.nearPlane },
                                Parameter { name: "farPlane"; value: mainCamera.farPlane },
                                Parameter { name: "width"; value: scene3d.width },
                                Parameter { name: "height"; value: scene3d.height }
                            ]
                        }
                        components: [ customMesh, materialPoint, meshTransform, layerPoints, picker ]
                    }

                    Q3D.Entity {
                        id: sphereEntity
                        property Layer layerSolid: Layer {
                                id: solidLayer
                            }
                        property ObjectPicker picker: ObjectPicker {
                            onClicked: console.log("Clicked sphere", pick.distance, pick.triangleIndex)
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
                        components: [ sphereMesh, materialPhong, meshTransform, layerSolid, picker ]
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
    }
    SystemPalette {
        id: palette
    }
}
