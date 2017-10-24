import QtQuick 2.7
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

import qt3deditorlib 1.0

Item {
    id: root
    property var currentEntitydata
    property var currentEntitydataTransform
    property alias visibleEntityItems: entityInstantiator.model
    property alias camera: mainCamera
    onVisibleEntityItemsChanged: {
        console.log("DBG: Visible Entities len: " + root.visibleEntityItems.length)
    }

    ColumnLayout {
        anchors.fill: parent
        //QCtl.ToolBar {
            Flow {
                topPadding: appStyle.controlMargin
                spacing: appStyle.controlMargin
                id: toolbar
                anchors.left: parent.left
                anchors.right: parent.right
                AxisGizmo {
                    Layout.topMargin: -appStyle.controlMargin
                    height: appStyle.controlHeightContainer
                    width: height
                    Layout.preferredHeight: toolbar.height
//                    Layout.preferredWidth: toolbar.height
                    finalTransform: mainCamera.viewMatrix
                }
                StyledLabel {
                    text: "PointSize: " + pointSizeSlider.value.toFixed(2)
                    verticalAlignment: Text.AlignVCenter
                }
                StyledCheckBox {
                    id: constantSizeCheckbox
                }
                StyledSlider {
                    id: pointSizeSlider
                    width: 60
                    value: 0.01
                    minimumValue: 0.01
                    maximumValue:  1.0
                }
                StyledLabel {
                    text: "Renderstyle:"
                    verticalAlignment: Text.AlignVCenter
                }
                StyledComboBox {
                    id: renderstyleSelect
                    model: [ "points", "discs", "surfel"]
                }
                StyledLabel {
                    text: "Color:"
                    verticalAlignment: Text.AlignVCenter
                }
                StyledComboBox {
                    id: colorizeSelect
                    model: [ "x", "y", "z"]//, "intensity"]
                }
                StyledLabel {
                    text: "Colorscale"
                    verticalAlignment: Text.AlignVCenter
                }
                StyledSlider {
                    id: colorscaleSlider
                    width: 60
                    value: 1.1
                    minimumValue: 0.1
                    maximumValue:  10.0
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://material/ic_settings"
                    onClicked: settings.show()
                }
                VisualizationSettings {
                    id: settings
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://material/ic_info"
                    onClicked: about.open()
                }
                About {
                    id: about
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
            onWheel: {
                cameraController.handleWheelScroll(wheel.angleDelta.y, Qt.point(wheel.x, wheel.y))
            }

            Item {
                id: priv
                property bool mouseOver: true
            }

            Scene3D {
                anchors.fill: parent
                id: scene3d
                aspects: ["render", "logic", "input"]
                //focus: priv.mouseOver
                Q3D.Entity {
                    id: sceneRoot
                    Camera {
                        id: mainCamera
                        projectionType: CameraLens.PerspectiveProjection
                        fieldOfView: 45
                        aspectRatio: scene3d.width/scene3d.height
                        nearPlane : 0.1*appStyle.cameraScale
                        farPlane : 1000.0*appStyle.cameraScale
                        position: Qt.vector3d( 0.0, 20.0, -40.0 )
                        upVector: Qt.vector3d( 0.0, 1.0, 0.0 )
                        viewCenter: Qt.vector3d( 0.0, 0.0, 0.0 )

                    }
                    EditorCameraController {
                        id: cameraController
                        camera: sceneView.camera
                        viewportSize: Qt.size(scene3d.width, scene3d.height)
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
                                                clearColor: appStyle.background3d
                                                NoDraw {}
                                            }
                                            LayerFilter {
                                                Layer {
                                                    id: noGridLayer
                                                }
                                                Layer {
                                                    id: gridLayer
                                                }
                                                layers: appStyle.showGrid ? gridLayer : noGridLayer
                                            }
                                            LayerFilter {
                                                layers: Layer {
                                                    id: solidLayer
                                                }
                                            }
                                            LayerFilter {
                                                layers: Layer {
                                                    id: pointLayer
                                                }
                                                TechniqueFilter {
                                                    id: techniqueFilter
                                                    matchAll: [
                                                        FilterKey { name: "primitiveType"; value: "point" },
                                                        FilterKey { name: "renderstyle";   value: renderstyleSelect.currentText }
                                                    ]
                                                    parameters: [
                                                        Parameter { name: "colorize"; value: colorizeSelect.currentIndex },
                                                        Parameter { name: "pointSize"; value: pointSizeSlider.value },
                                                        Parameter { name: "fieldOfView"; value: mainCamera.fieldOfView },
                                                        Parameter { name: "fieldOfViewVertical"; value: mainCamera.fieldOfView/mainCamera.aspectRatio },
                                                        Parameter { name: "nearPlane"; value: mainCamera.nearPlane },
                                                        Parameter { name: "farPlane"; value: mainCamera.farPlane },
                                                        Parameter { name: "width"; value: scene3d.width },
                                                        Parameter { name: "height"; value: scene3d.height },
                                                        Parameter { name: "lod"; value: appStyle.pointcloudLod },
                                                        Parameter { name: "colorscale"; value: colorscaleSlider.value },
                                                        Parameter { name: "constantSize"; value: constantSizeCheckbox.checked },
                                                        Parameter { name: "yPointsUp"; value: appStyle.coordinateSystemYPointsUp }

                                                    ]
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
                            }
                        },
                        // Event Source will be set by the Qt3DQuickWindow
                        InputSettings {
                            objectName: "inputSettings"
                            eventSource: scene3d
                            enabled: true
                        },
                        Q3D.Transform {
                            id: worldTransform
                            scale: appStyle.cameraScale
                        }
                    ]

                    Q3D.Entity {
                        id: gridEntity
                        Q3D.Transform {
                            id: gridTransform
                            translation: Qt.vector3d(0, 0, 0)
                        }
                        HelperGridMesh {
                            id: gridMesh
                            gridSpacing: appStyle.gridSpacing
                            lines: appStyle.gridLines
                        }

                        property Material materialPhong: PhongMaterial {
                            ambient: Qt.rgba(0.0,0.0,0.0,1.0)
                        }
                        components: [ gridMesh, materialPhong, gridTransform, gridLayer ]
                    }

                    Q3D.Entity {
                        id: objectsRoot

                        components: [
                            Q3D.Transform {
                                id: coordianteSystemTransform
                                rotationX: appStyle.coordinateSystemYPointsUp ? 0 : -90//Math.PI*0.5
                            }]
                        UPNS.PointcloudCoordinatesystem {
                            id: coordSys
                        }
                        Q3D.NodeInstantiator {
                            id: entityInstantiator
                            model: root.visibleEntityItems
                            delegate: MapitEntity {
                                mainCameratmp: mainCamera
                                scene3dtmp: scene3d
                                coordinateSystem: coordSys
                                //transformMat: root.currentEntitydataTransform ? root.currentEntitydataTransform.matrix : Qt.matrix4x4(1, 0, 0, 0,
                                //                                                                                                      0, 1, 0, 0,
                                //                                                                                                      0, 0, 1, 0,
                                //                                                                                                      0, 0, 0, 1)
                                layer: pointLayer
                                parametersTmp: techniqueFilter.parameters
                                currentCheckout: UPNS.Checkout {
                                    id: co
                                    repository: globalRepository
                                    name: model.checkoutName
                                }
                                currentEntitydata: UPNS.Entitydata {
                                    checkout: co
                                    path: model.path
                                }
                            }
                        }

//                        Q3D.Entity {
//                            id: sphereEntity
//                            property ObjectPicker picker: ObjectPicker {
//                                onClicked: console.log("Clicked sphere", pick.distance, pick.triangleIndex)
//                            }

//                            property var meshTransform: Q3D.Transform {
//                                id: sphereTransform
//                                property real userAngle: 0.0
//                                translation: Qt.vector3d(20, 0, 0)
//                                rotation: fromAxisAndAngle(Qt.vector3d(0, 1, 0), userAngle)
//                            }
//                            property var sphereMesh: SphereMesh {
//                                radius: 4
//                            }
//                            property Material materialPhong: PhongMaterial { }
//                            components: [ sphereMesh, materialPhong, meshTransform, layerSolid, picker ]
//                            NumberAnimation {
//                                target: sphereTransform
//                                property: "userAngle"
//                                duration: 10000
//                                from: 0
//                                to: 360

//                                loops: Animation.Infinite
//                                running: true
//                            }
//                        }
                    }
                }
            }
        }
    }
}
