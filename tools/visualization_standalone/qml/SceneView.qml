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
    property var applicationState
    property var currentEntitydata
    Connections {
        target: currentEntitydata
        onUpdated: {
            colorizeSelect.model = currentEntitydata.getInfo("fields")
        }
    }

    property var currentEntitydataTransform
    property alias visibleEntityItems: entityInstantiator.model
    property alias camera: mainCamera
    property alias currentFrameId: frameIdChooser.currentText
    onVisibleEntityItemsChanged: {
        console.log("DBG: Visible Entities len: " + root.visibleEntityItems.length)
    }

    ColumnLayout {
        anchors.fill: parent
        Flow {
            topPadding: appStyle.controlMargin
            spacing: appStyle.controlMargin
            id: toolbar
            anchors.left: parent.left
            anchors.right: parent.right
            RowLayout {
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
            }
            RowLayout {
                StyledLabel {
                    text: "Renderstyle:"
                    verticalAlignment: Text.AlignVCenter
                }
                StyledComboBox {
                    id: renderstyleSelect
                    model: [ "points", "discs", "surfel"]
                }
            }
            RowLayout {
                StyledLabel {
                    text: "Color:"
                    verticalAlignment: Text.AlignVCenter
                }
                StyledComboBox {
                    id: colorizeSelect
                    model: currentEntitydata.getInfo("fields")//[ "x", "y", "z"]//, "intensity"]
                    textRole: "name"
                }
                StyledComboBox {
                    id: colorModeSelect
                    model: ["flashlight (double sided)", "Axis", "Axis (HSV)", "flashlight (single sided)", "flashlight2"]
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
            }

            RowLayout {
                StyledLabel {
                    text: "Ref. Frame:"
                    verticalAlignment: Text.AlignVCenter
                }
                StyledComboBox {
                    id: frameIdChooser
                    model: []
                    function addUniqueFrameIds(frameIds) {
                        var modelArray = frameIdChooser.model
                        frameIds.forEach(function(frameId) {
                            var found = false
                            for(var i=0; i<model.length; ++i) {
                                var name = model[i]
                                if(name === frameId) {
                                    found = true
                                }
                            }
                            if(!found) modelArray.push(frameId)
                        })
                        frameIdChooser.model = modelArray
                    }
                }
            }
            RowLayout {
                StyledButton {
                    isIcon: true
                    iconSource: "image://primitive/sphere-skinny"
                    onClicked: applicationState.selectOperator("place_primitive", {type:"sphere"})
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://primitive/plane-skinny"
                    onClicked: applicationState.selectOperator("place_primitive", {type:"plane"})
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://primitive/cylinder-skinny"
                    onClicked: applicationState.selectOperator("place_primitive", {type:"cylinder"})
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://primitive/cone-skinny"
                    onClicked: applicationState.selectOperator("place_primitive", {type:"cone"})
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://primitive/torus-skinny"
                    onClicked: applicationState.selectOperator("place_primitive", {type:"torus"})
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://primitive/cube-skinny"
                    onClicked: applicationState.selectOperator("place_primitive", {type:"cube"})
                }
            }
            RowLayout {
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
            }
            About {
                id: about
            }
        }

        MouseArea {
            Layout.fillWidth: true
            Layout.fillHeight: true
            id: sceneMouseArea

            hoverEnabled: true

            onEntered: priv.mouseOver = true
            onExited: priv.mouseOver = false
            onWheel: {
                cameraController.handleWheelScroll(wheel.angleDelta.y, Qt.point(wheel.x, wheel.y))
            }

            Item {
                id: priv
                property bool mouseOver: true
            }

            StyledButton {
                z: 10
                anchors.bottom: parent.bottom
                anchors.left: parent.left
                width: height
                checkable: true
                checked: appStyle.showGizmoAlways
                clip: true
                onCheckedChanged: appStyle.showGizmoAlways = checked
                AxisGizmo {
                    anchors.fill: parent
                    finalTransform: mainCamera.viewMatrix.times(coordianteSystemTransform.matrix)
                }
            }

            UPNS.RayCast {
                id: mouseRaycast
                viewMatrix: camera.viewMatrix
                projectionMatrix: camera.projectionMatrix
                viewportSize: Qt.size(scene3d.width, scene3d.height)
                pointOnPlane: camera.viewCenter
                planeNormal: camera.viewVector
                screenPosition: Qt.vector2d(sceneMouseArea.mouseX, sceneMouseArea.mouseY)
            }

            Q3D.Transform {
                id: previewTransform
                translation: coordianteSystemTransform.matrix.inverted().times(mouseRaycast.worldPosition)
                onMatrixChanged: {
                    appStyle.tmpPreviewMatrix = matrix
                }
            }

            Scene3D {
                //hoverEnabled: true
                anchors.fill: parent
                id: scene3d
                aspects: ["render", "logic", "input"]
                focus: priv.mouseOver
                z: 9
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
                        onViewCenterChanged: gizmoHideTimer.start()
                        Timer {
                            id: gizmoHideTimer
                            interval: 2000
                            onRunningChanged: gizmoEntity.isVisible = running
                        }
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
                                                    id: invisibleLayer
                                                }
                                                NoDraw {}
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
                                                    function fieldnameToShaderindex(text) {
                                                        if(text === "x") return 0;
                                                        if(text === "y") return 1;
                                                        if(text === "z") return 2;
                                                        if(text === "normal_x") return 3;
                                                        if(text === "normal_y") return 4;
                                                        if(text === "normal_z") return 5;
                                                        if(text === "curvature") return 6;
                                                        if(text === "rgb") return 7;
                                                        if(text === "intensity") return 8;
                                                    }

                                                    matchAll: [
                                                        FilterKey { name: "primitiveType"; value: "point" },
                                                        FilterKey { name: "renderstyle";   value: renderstyleSelect.currentText }
                                                    ]
                                                    parameters: [
                                                        Parameter { name: "colorize"; value: techniqueFilter.fieldnameToShaderindex(colorizeSelect.currentText) },
                                                        Parameter { name: "colorMode"; value: colorModeSelect.currentIndex },
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
                                            LayerFilter {
                                                Layer {
                                                    id: gizmoLayer
                                                }
                                                layers: gizmoLayer
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

                        }
                    ]

                    Q3D.Entity {
                        id: worldEntity
                        components: [
                            Q3D.Transform {
                                id: worldTransform
                                scale: appStyle.cameraScale
                            }]
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
                            id: gizmoEntity

                            property var meshTransform: Q3D.Transform {
                                id: viewCenterTransform
                                translation: mainCamera.viewCenter
                                rotation: coordianteSystemTransform.rotation
                            }
                            PerVertexColorMaterial {
                                id: perVertexColorMaterial
                            }
                            GeometryRenderer {
                                id: gizmoRenderer

                                function rebuild() {
                                    buffer.data = buffer.buildGrid()
                                }
                                instanceCount: 1
                                indexOffset: 0
                                firstInstance: 0
                                vertexCount: 6
                                primitiveType: GeometryRenderer.Lines
                                geometry: Geometry {
                                    Attribute {
                                        id: positionAttribute
                                        attributeType: Attribute.VertexAttribute
                                        vertexBaseType: Attribute.Float
                                        vertexSize: 3
                                        byteOffset: 0
                                        byteStride: 6 * 4
                                        count: 6
                                        name: "vertexPosition"//defaultPositionAttributeName()
                                        buffer: Buffer {
                                            id: bufferPosColor
                                            type: Buffer.VertexBuffer
                                            function buildGrid() {
                                                var vertices = 6;
                                                var vertexFloats = 6;
                                                var vertexArray = new Float32Array(vertexFloats * vertices);
                                                vertexArray[ 0] =-1.0; vertexArray[ 1] = 0.0; vertexArray[ 2] = 0.0
                                                vertexArray[ 3] = 1.0; vertexArray[ 4] = 0.0; vertexArray[ 5] = 0.0
                                                vertexArray[ 6] = 1.0; vertexArray[ 7] = 0.0; vertexArray[ 8] = 0.0
                                                vertexArray[ 9] = 1.0; vertexArray[10] = 0.0; vertexArray[11] = 0.0

                                                vertexArray[12] = 0.0; vertexArray[13] =-1.0; vertexArray[14] = 0.0
                                                vertexArray[15] = 0.0; vertexArray[16] = 1.0; vertexArray[17] = 0.0
                                                vertexArray[18] = 0.0; vertexArray[19] = 1.0; vertexArray[20] = 0.0
                                                vertexArray[21] = 0.0; vertexArray[22] = 1.0; vertexArray[23] = 0.0

                                                vertexArray[24] = 0.0; vertexArray[25] = 0.0; vertexArray[26] =-1.0
                                                vertexArray[27] = 0.0; vertexArray[28] = 0.0; vertexArray[29] = 1.0
                                                vertexArray[30] = 0.0; vertexArray[31] = 0.0; vertexArray[32] = 1.0
                                                vertexArray[33] = 0.0; vertexArray[34] = 0.0; vertexArray[35] = 1.0
                                                return vertexArray
                                            }
                                            data: buildGrid()
                                        }
                                    }
                                    Attribute {
                                        id: colorAttribute
                                        attributeType: Attribute.VertexAttribute
                                        vertexBaseType: Attribute.Float
                                        vertexSize: 3
                                        byteOffset: 3 * 4
                                        byteStride: 6 * 4
                                        count: 6
                                        name: "vertexColor"//defaultColorAttributeName()
                                        buffer: bufferPosColor
                                    }
                                }
                            }
                            property bool isVisible
                            property Layer currentLayer: isVisible || appStyle.showGizmoAlways ? solidLayer : invisibleLayer
                            components: [ gizmoRenderer, perVertexColorMaterial, viewCenterTransform, currentLayer ]
                        }

                        Q3D.Entity {
                            id: objectsRoot
                            property alias objectsRootTransform: coordianteSystemTransform
                            components: [
                                Q3D.Transform {
                                    id: coordianteSystemTransform
                                    rotationX: appStyle.coordinateSystemYPointsUp ? 0 : -90//Math.PI*0.5
                                }]

                            HandleTranslate {
                                id: manipulationGizmo
                                layer: gizmoLayer
                                property var gizmoTransform: Q3D.Transform {
                                    translation: Qt.vector3d(2.0,2.0,2.0)
                                }
                                components: [ gizmoTransform, gizmoLayer ]
                            }

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
                                    currentFrameId: root.currentFrameId
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
                                        Component.onCompleted: frameIdChooser.addUniqueFrameIds(co.getFrameIds())
                                    }
                                    currentEntitydata: UPNS.Entitydata {
                                        checkout: co
                                        path: model.path
                                    }
                                }
                            }
                            Q3D.Entity {
                                id: annotationPreviewEntity
                                property ObjectPicker picker: ObjectPicker {
                                    onClicked: console.log("Clicked sphere", pick.distance, pick.triangleIndex)
                                }

                                property var meshTransform: Q3D.Transform {
                                    matrix: appStyle.tmpPreviewMatrix
                                }
                                property var sphereMesh: SphereMesh { }
                                property var planeMesh: PlaneMesh { }
                                property var cylinderMesh: CylinderMesh { }
                                property var coneMesh: ConeMesh { }
                                property var torusMesh: TorusMesh { minorRadius: 0.3 }
                                property var cubeMesh: CuboidMesh { }
                                property var selectedMesh: appStyle.tmpPrimitiveType === "sphere" ? sphereMesh
                                                         : appStyle.tmpPrimitiveType === "plane" ? planeMesh
                                                         : appStyle.tmpPrimitiveType === "cylinder" ? cylinderMesh
                                                         : appStyle.tmpPrimitiveType === "cone" ? coneMesh
                                                         : appStyle.tmpPrimitiveType === "torus" ? torusMesh
                                                         : appStyle.tmpPrimitiveType === "cube" ? cubeMesh
                                                         : sphereMesh

                                property var materialPhong: PhongMaterial { }
                                property Layer currentLayer: appStyle.tmpPlacePrimitive ? solidLayer : invisibleLayer
                                components: [ selectedMesh, materialPhong, meshTransform, currentLayer, picker ]
                            }
                        }
                    }
                }
            }
        }
    }
}
