import QtQuick 2.7
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import "qrc:/qml/scene"
import QtQuick.Scene3D 2.0
import Qt3D.Core 2.0 as Q3D
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0
import QtQml.Models 2.3

import pcl 1.0

import fhac.upns 1.0 as UPNS

import QtQuick 2.0 as QQ2

import qt3deditorlib 1.0

import "components"
import "dialogs"
import "qrc:/qml/network"

import FileIO 1.0
import Library 1.0
import Clipboard 1.0

Item {
    id: root
    property var currentEntitydata: globalApplicationState.currentEntitydata
    //    Connections {
    //        target: currentEntitydata
    //        onUpdated: {
    //            colorizeSelect.model = currentEntitydata.info["fields"]
    //        }
    //    }

    property real pointSize: pointSizeSlider.value
    property int shaderVar: techniqueFilter.fieldnameToShaderindex(colorizeSelect.currentText)
    property int shaderVar2: colorModeSelect.currentIndex //TODO: shaderVar and Var2 are temporary for demo
    property string renderStyle: renderstyleSelect.currentText
    property var currentCheckout: globalApplicationState.currentCheckout
    property var currentEntitydataTransform: globalApplicationState.currentEntityTransform
    property ListModel visibleEntityModel//: entityInstantiator.model
    property var allVisualInfoModel
    property alias camera: mainCamera
    property alias currentFrameId: frameIdChooser.currentText
    property MapitClient mapitClient: globalApplicationState.mapitClient
    onVisibleEntityModelChanged: {
        console.log("DBG: Visible Entities len: " + root.visibleEntityModel.count)
    }

    ColumnLayout {
        anchors.fill: parent
        Flow {
            topPadding: appStyle.controlMargin
            spacing: appStyle.controlMargin
            id: toolbar
            anchors.left: parent.left
            anchors.right: parent.right
            z: 1000
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
                    model: currentEntitydata.info["fields"] //currentEntitydata.info["fields"]//[ "x", "y", "z"]//, "intensity"]
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
                FrameIdChooser {
                    Layout.fillWidth: true
                    Layout.minimumWidth: 100
                    id: frameIdChooser
                    allowNew: false
                    //currentCheckout: root.currentCheckout
                }

                //                StyledComboBox {
                //                    id: frameIdChooser
                //                    model: []
                //                    function addUniqueFrameIds(frameIds) {
                //                        var modelArray = frameIdChooser.model
                //                        frameIds.forEach(function(frameId) {
                //                            var found = false
                //                            for(var i=0; i<model.length; ++i) {
                //                                var name = model[i]
                //                                if(name === frameId) {
                //                                    found = true
                //                                }
                //                            }
                //                            if(!found) modelArray.push(frameId)
                //                        })
                //                        frameIdChooser.model = modelArray
                //                    }
                //                }
            }
            RowLayout {
                StyledButton {
                    isIcon: true
                    iconSource: "image://primitive/sphere-skinny"
                    tooltip: "Place <i>sphere</i>"
                    onClicked: {
                        globalApplicationState.currentDetailDialog = "../pipelines/place_primitive"
                        appStyle.tmpPrimitiveType = "sphere"
                    }
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://primitive/plane-skinny"
                    tooltip: "Place <i>plane</i>"
                    onClicked: {
                        globalApplicationState.currentDetailDialog = "../pipelines/place_primitive"
                        appStyle.tmpPrimitiveType = "plane"
                    }
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://primitive/cylinder-skinny"
                    tooltip: "Place <i>cylinder</i>"
                    onClicked: {
                        globalApplicationState.currentDetailDialog = "../pipelines/place_primitive"
                        appStyle.tmpPrimitiveType = "cylinder"
                    }
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://primitive/cone-skinny"
                    tooltip: "Place <i>cone</i>"
                    onClicked: {
                        globalApplicationState.currentDetailDialog = "../pipelines/place_primitive"
                        appStyle.tmpPrimitiveType = "cone"
                    }
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://primitive/torus-skinny"
                    tooltip: "Place <i>torus</i>"
                    onClicked: {
                        globalApplicationState.currentDetailDialog = "../pipelines/place_primitive"
                        appStyle.tmpPrimitiveType = "torus"
                    }
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://primitive/cube-skinny"
                    tooltip: "Place <i>cube</i>"
                    onClicked: {
                        globalApplicationState.currentDetailDialog = "../pipelines/place_primitive"
                        appStyle.tmpPrimitiveType = "cube"
                    }
                }
                StyledButton {
                    id: btnClick
                    isIcon: true
                    checkable: true
                    iconSource: "image://material/ic_location_searching"
                    tooltip: "Drag Annotation with mouse directly into the scene"
                    onCheckedChanged: {
                        appStyle.tmpFollowMouse = checked
                    }
                    Connections {
                        target: appStyle
                        onTmpFollowMouseChanged:
                            btnClick.checked = appStyle.tmpFollowMouse
                    }
                }
            }
            StyledButton {
                id: boundingBoxButton
                width: height
                text: "BB"
                tooltip: "Show <i>Bounding Boxes</i>"
                checkable: true
            }
            RowLayout {
                StyledButton {
                    isIcon: true
                    iconSource: "image://material/ic_settings"
                    tooltip: "Settings"
                    onClicked: settings.show()
                }
                VisualizationSettings {
                    id: settings
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://material/ic_info"
                    tooltip: "About"
                    onClicked: about.open()
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://material/ic_3d_rotation"
                    enabled: mapitScene.entityInstantiator.loadingItems == 0
                    tooltip: "Top View"
                    onClicked: {
                        var bbCenter = mapitScene.entityInstantiator.boundingboxMax.plus(mapitScene.entityInstantiator.boundingboxMin).times(0.5)
                        var offset = bbCenter.minus(mainCamera.position)
                        var direction = offset.normalized().times(0.01)
                        mainCamera.upVector = Qt.vector3d(0.0, 1.0, -0.001).plus(direction)
                        centerAnimation.start()
                        camPosAnimation.start()
                    }
                    Vector3dAnimation {
                        property real bbEdgeX: mapitScene.entityInstantiator.boundingboxMax.x - mapitScene.entityInstantiator.boundingboxMin.x
                        property real bbEdgeY: mapitScene.entityInstantiator.boundingboxMax.y - mapitScene.entityInstantiator.boundingboxMin.y
                        property real bbEdgeZ: mapitScene.entityInstantiator.boundingboxMax.z - mapitScene.entityInstantiator.boundingboxMin.z
                        property real bbEdgeMax: Math.max(bbEdgeX, Math.max(bbEdgeY, bbEdgeZ))
                        id: camPosAnimation
                        from: mainCamera.position
                        to: mapitScene.entityInstantiator.boundingboxMax.plus(mapitScene.entityInstantiator.boundingboxMin).times(0.5)
                        .plus(Qt.vector3d(0.0, Math.max(10.0, bbEdgeMax / (2 * Math.tan(mainCamera.fieldOfView * 0.5))), 0.0))
                        target: mainCamera
                        property: "position"
                        duration: 1000
                        easing.type: Easing.InOutQuad
                    }
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://material/ic_filter_center_focus"
                    enabled: mapitScene.entityInstantiator.loadingItems == 0
                    tooltip: "Center Data"
                    onClicked: {
                        mainCamera.upVector = Qt.vector3d(0.0, 1.0, -0.0001)
                        centerAnimation.start()
                    }
                    Vector3dAnimation {
                        id: centerAnimation
                        from: mainCamera.viewCenter
                        to: mapitScene.entityInstantiator.boundingboxMax.plus(mapitScene.entityInstantiator.boundingboxMin).times(0.5)
                        target: mainCamera
                        property: "viewCenter"
                        duration: 1000
                        easing.type: Easing.InOutQuad
                    }
                }
                StyledButton {
                    isIcon: true
                    iconSource: "image://material/ic_replay"
                    tooltip: "Reset Camera"
                    onClicked: {
                        mainCamera.position = Qt.vector3d( 0.0, 20.0, -40.0 )
                        mainCamera.viewCenter = Qt.vector3d( 0.0, 0.0, 0.0 )
                    }
                }
            }
            About {
                id: about
            }
        }

        ColumnLayout {
            StyledSplitView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                orientation: Qt.Vertical
                Item {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    z: 0
                    MouseArea {
                        anchors.fill: parent
                        id: sceneMouseArea
                        z: 100

                        hoverEnabled: true
                        preventStealing: true
                        propagateComposedEvents: true

                        onEntered: priv.mouseOver = true
                        onExited: priv.mouseOver = false
                        onWheel: {
                            cameraController.handleWheelScroll(wheel.angleDelta.y, Qt.point(wheel.x, wheel.y))
                        }
                        acceptedButtons: appStyle.captureMouse ? Qt.LeftButton : Qt.NoButton
                        onClicked: {
                            mouse.accepted = false
                            appStyle.emitClickedAction(mouse)
                            appStyle.tmpFollowMouse = false
                        }

                        Item {
                            id: priv
                            property bool mouseOver: true
                        }
                    }

                    StyledButton {
                        z: 200
                        anchors.bottom: parent.bottom
                        anchors.left: parent.left
                        width: height
                        checkable: true
                        checked: appStyle.showGizmoAlways
                        clip: true
                        onCheckedChanged: appStyle.showGizmoAlways = checked
                        AxisGizmo {
                            anchors.fill: parent
                            finalTransform: mainCamera.viewMatrix.times(mapitScene.coordianteSystemTransform.matrix)
                        }
                    }

                    QCtl.BusyIndicator {
                        z: 200
                        anchors.bottom: parent.bottom
                        anchors.right: parent.right
                        running: mapitScene.entityInstantiator.loadingItems != 0
                        StyledButton {
                            visible: parent.running
                            anchors.fill: parent
                            opacity: 0.0
                            tooltip: "Calculating bounding boxes, ..."
                        }
                    }

                    UPNS.Raycast {
                        id: mouseRaycast
                        viewMatrix: camera.viewMatrix
                        projectionMatrix: camera.projectionMatrix
                        viewportSize: Qt.size(scene3d.width, scene3d.height)
                        pointOnPlane: camera.viewCenter
                        planeNormal: camera.viewVector
                        screenPosition: Qt.vector2d(sceneMouseArea.mouseX, sceneMouseArea.mouseY)
                    }
                    Repeater {
                        model: root.mapitClient ? root.mapitClient.state.realtimeObjects.count : 0
                        Text {
                            z: 1000
                            text: peerRaycast.theRto ? root.mapitClient.state.peerToPeerState[peerRaycast.theRto.peerOwner].peername : "unknown"
                            x: peerRaycast.screenPosition.x
                            y: peerRaycast.screenPosition.y
                            UPNS.Raycast {
                                id: peerRaycast
                                viewMatrix: camera.viewMatrix
                                projectionMatrix: camera.projectionMatrix
                                viewportSize: Qt.size(scene3d.width, scene3d.height)
                                property var theRto: root.mapitClient.state.realtimeObjects.get(index)
                                property matrix4x4 posMat: theRto.tf.inverted()
                                worldPosition: Qt.vector3d(posMat.m14, posMat.m24, posMat.m34)
                            }
                        }
                    }

                    Q3D.Transform {
                        id: previewTransform
                        translation: mapitScene.coordianteSystemTransform.matrix.inverted().times(mouseRaycast.worldPosition)
                        onMatrixChanged: {
                            if(appStyle.tmpFollowMouse) {
                                appStyle.tmpPreviewMatrix = matrix
                            }
                        }
                    }
                    Scene3D {
                        anchors.fill: parent
//                        hoverEnabled: true
//                        focus: true
                        id: scene3d
                        aspects: ["render", "logic", "input"]
                        //focus: priv.mouseOver
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
                                upVector: Qt.vector3d( 0.0, 1.0, 0.0 ) // small offset to make top view easily possible
                                viewCenter: Qt.vector3d( 0.0, 0.0, 0.0 )
                                onViewCenterChanged: gizmoHideTimer.start()
                                Timer {
                                    id: gizmoHideTimer
                                    interval: 2000
                                    onRunningChanged: mapitScene.gizmoEntity.isVisible = running
                                }
                            }
                            // TODO: A Qml configured cameracontroller should be implemented. See "simple-qml" example. At the time of this writing,
                            // the sample used "OrbitCameraController" instead of CameraController.
//                          CameraController {
//                                id: cameraController
//                                camera: sceneView.camera
//                                //viewportSize: Qt.size(scene3d.width, scene3d.height)
//                            }

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
                            MapitScene {
                                id: mapitScene
                                visibleEntityPaths: root.visibleEntityModel
                                allEntities: root.allVisualInfoModel
                                currentFrameId: root.currentFrameId
                                currentCheckout: globalApplicationState.currentCheckout
                                mapitClient: root.mapitClient
                                camera: mainCamera
                                appStyleOptional: appStyle

                                gridLayer: gridLayer
                                gizmoLayer: gizmoLayer
                                invisibleLayer: invisibleLayer
                                pointLayer: pointLayer
                                solidLayer: solidLayer
                            }

//                            Q3D.Entity {
//                                id: worldEntity
//                                components: [
//                                    Q3D.Transform {
//                                        id: worldTransform
//                                        scale: appStyle.cameraScale
//                                    }]
//                                Q3D.Entity {
//                                    id: gridEntity
//                                    Q3D.Transform {
//                                        id: gridTransform
//                                        translation: Qt.vector3d(0, 0, 0)
//                                    }
//                                    HelperGridMesh {
//                                        id: gridMesh
//                                        gridSpacing: appStyle.gridSpacing
//                                        lines: appStyle.gridLines
//                                    }

//                                    property Material materialPhong: PhongMaterial {
//                                        ambient: Qt.rgba(0.0,0.0,0.0,1.0)
//                                    }
//                                    components: [ gridMesh, materialPhong, gridTransform, gridLayer ]
//                                }
//                                Q3D.NodeInstantiator {
//                                    id: realtimeObjectInstantiator
//                                    model: root.mapitClient ? root.mapitClient.state.realtimeObjects.count : 0
//                                    HandleFrustum {
//                                        id: peerGizmo
//                                        layer: gizmoLayer
//                                        property var theRto: root.mapitClient.state.realtimeObjects.get(index)
//                                        property var gizmoTransform: Q3D.Transform {
//                                            matrix: peerGizmo.theRto ? peerGizmo.theRto.tf.inverted() : Qt.matrix4x4()
//                                        }
//                                        aspectRatio: theRto.additionalData.aspect
//                                        horizontalFov: theRto.additionalData.fov
//                                        components: [ gizmoTransform ]

////                                        Q3D.Entity {
////                                            property Material material: PhongMaterial { diffuse: "red" }
////                                            components: [text1Transform, solidLayer, material, text]
////                                            ExtrudedTextMesh { // Not a type
////                                                id: text
////                                                text: "Hello World"
////                                                width: 20
////                                                height: 10
////                                            }

////                                            Q3D.Transform {
////                                                id: text1Transform
////                                                translation: Qt.vector3d(0, 2, 0)//manipulationGizmo.gizmoTransform.pos.plus( Qt.vector3d(0, 2, 0) )
////                                            }
////                                            TextLabelEntity { // Error when more than 2 peers
////                                                text: manipulationGizmo.theRto ? root.mapitClient.state.peerToPeerState[manipulationGizmo.theRto.peerOwner].peername : "unknown"
////                                                layer: solidLayer //TO DO: would be awesome if children are rendered, when parent is in "solidLayer" automatically
////                                            }
////                                        }
//                                    }
////                                    Q3D.Entity {

////                                        property var meshTransform: Q3D.Transform {
////                                            property var theRto: root.mapitClient.state.realtimeObjects.get(index)
////                                            matrix: theRto ? theRto.tf.inverted() : Qt.matrix4x4()
////                                        }
////                                        property var coneMesh: ConeMesh { }
////                                        components: [ coneMesh, perVertexColorMaterial, meshTransform, solidLayer ]
////                                    }
//                                }

//                                Q3D.Entity {
//                                    id: gizmoEntity

//                                    property var meshTransform: Q3D.Transform {
//                                        id: viewCenterTransform
//                                        translation: mainCamera.viewCenter
//                                        rotation: coordianteSystemTransform.rotation
//                                    }
//                                    PerVertexColorMaterial {
//                                        id: perVertexColorMaterial
//                                    }
//                                    GeometryRenderer {
//                                        id: gizmoRenderer

//                                        function rebuild() {
//                                            buffer.data = buffer.buildGrid()
//                                        }
//                                        instanceCount: 1
//                                        indexOffset: 0
//                                        firstInstance: 0
//                                        vertexCount: 6
//                                        primitiveType: GeometryRenderer.Lines
//                                        geometry: Geometry {
//                                            Attribute {
//                                                id: positionAttribute
//                                                attributeType: Attribute.VertexAttribute
//                                                vertexBaseType: Attribute.Float
//                                                vertexSize: 3
//                                                byteOffset: 0
//                                                byteStride: 6 * 4
//                                                count: 6
//                                                name: "vertexPosition"//defaultPositionAttributeName()
//                                                buffer: Buffer {
//                                                    id: bufferPosColor
//                                                    type: Buffer.VertexBuffer
//                                                    function buildGrid() {
//                                                        var vertices = 6;
//                                                        var vertexFloats = 6;
//                                                        var vertexArray = new Float32Array(vertexFloats * vertices);
//                                                        vertexArray[ 0] =-1.0; vertexArray[ 1] = 0.0; vertexArray[ 2] = 0.0
//                                                        vertexArray[ 3] = 1.0; vertexArray[ 4] = 0.0; vertexArray[ 5] = 0.0
//                                                        vertexArray[ 6] = 1.0; vertexArray[ 7] = 0.0; vertexArray[ 8] = 0.0
//                                                        vertexArray[ 9] = 1.0; vertexArray[10] = 0.0; vertexArray[11] = 0.0

//                                                        vertexArray[12] = 0.0; vertexArray[13] =-1.0; vertexArray[14] = 0.0
//                                                        vertexArray[15] = 0.0; vertexArray[16] = 1.0; vertexArray[17] = 0.0
//                                                        vertexArray[18] = 0.0; vertexArray[19] = 1.0; vertexArray[20] = 0.0
//                                                        vertexArray[21] = 0.0; vertexArray[22] = 1.0; vertexArray[23] = 0.0

//                                                        vertexArray[24] = 0.0; vertexArray[25] = 0.0; vertexArray[26] =-1.0
//                                                        vertexArray[27] = 0.0; vertexArray[28] = 0.0; vertexArray[29] = 1.0
//                                                        vertexArray[30] = 0.0; vertexArray[31] = 0.0; vertexArray[32] = 1.0
//                                                        vertexArray[33] = 0.0; vertexArray[34] = 0.0; vertexArray[35] = 1.0
//                                                        return vertexArray
//                                                    }
//                                                    data: buildGrid()
//                                                }
//                                            }
//                                            Attribute {
//                                                id: colorAttribute
//                                                attributeType: Attribute.VertexAttribute
//                                                vertexBaseType: Attribute.Float
//                                                vertexSize: 3
//                                                byteOffset: 3 * 4
//                                                byteStride: 6 * 4
//                                                count: 6
//                                                name: "vertexColor"//defaultColorAttributeName()
//                                                buffer: bufferPosColor
//                                            }
//                                        }
//                                    }
//                                    property bool isVisible
//                                    property Layer currentLayer: isVisible || appStyle.showGizmoAlways ? solidLayer : invisibleLayer
//                                    components: [ gizmoRenderer, perVertexColorMaterial, viewCenterTransform, currentLayer ]
//                                }

//                                Q3D.Entity {
//                                    id: objectsRoot
//                                    property alias objectsRootTransform: coordianteSystemTransform
//                                    components: [
//                                        Q3D.Transform {
//                                            id: coordianteSystemTransform
//                                            rotationX: appStyle.coordinateSystemYPointsUp ? 0 : -90//Math.PI*0.5
//                                        }]

//                                    //                                    HandleTranslate {
//                                    //                                        id: manipulationGizmo
//                                    //                                        layer: gizmoLayer
//                                    //                                        property var gizmoTransform: Q3D.Transform {
//                                    //                                            translation: Qt.vector3d(2.0,2.0,2.0)
//                                    //                                        }
//                                    //                                        components: [ gizmoTransform, gizmoLayer ]
//                                    //                                    }

//                                    UPNS.PointcloudCoordinatesystem {
//                                        id: coordSys
//                                    }
//                                    Timer {
//                                        id: boundingBoxRecalculator
//                                        interval: 1
//                                        repeat: false
//                                        onTriggered: {
//                                            var minx= Infinity, miny= Infinity, minz= Infinity
//                                            , maxx=-Infinity, maxy=-Infinity, maxz=-Infinity
//                                            for(var i=0; i<entityInstantiator.count; i++) {
//                                                var item = entityInstantiator.objectAt(i)
//                                                if(minx > item.min.x) minx = item.min.x
//                                                if(miny > item.min.y) miny = item.min.y
//                                                if(minz > item.min.z) minz = item.min.z
//                                                if(maxx < item.max.x) maxx = item.max.x
//                                                if(maxy < item.max.y) maxy = item.max.y
//                                                if(maxz < item.max.z) maxz = item.max.z
//                                            }
//                                            entityInstantiator.boundingboxMin = Qt.vector3d(minx, miny, minz)
//                                            entityInstantiator.boundingboxMax = Qt.vector3d(maxx, maxy, maxz)
//                                        }
//                                    }

//                                    Q3D.NodeInstantiator {
//                                        id: entityInstantiator
//                                        function recalcBoundingBox() {
//                                            boundingBoxRecalculator.start()
//                                        }
//                                        property int loadingItems: 0
//                                        property vector3d boundingboxMin // use recalc. Propertybindings would not reevaluate if model changed
//                                        property vector3d boundingboxMax // use recalc. Propertybindings would not reevaluate if model changed

//                                        model: root.visibleEntityModel
//                                        onModelChanged: recalcBoundingBox()
//                                        onObjectAdded: recalcBoundingBox()
//                                        onObjectRemoved: recalcBoundingBox()
//                                        //model: root.visibleEntityModel
//                                        delegate: MapitEntity {
//                                            mainCameratmp: mainCamera
//                                            scene3dtmp: scene3d
//                                            coordinateSystem: coordSys
//                                            currentFrameId: root.currentFrameId
//                                            layer: pointLayer
//                                            //parametersTmp: techniqueFilter.parameters
//                                            //Currently only one checkout is supported
//                                            currentCheckout: globalApplicationState.currentCheckout
//                                            //                                    currentCheckout: UPNS.Checkout {
//                                            //                                        id: co
//                                            //                                        repository: globalRepository
//                                            //                                        name: model.checkoutName
//                                            //                                        //Component.onCompleted: frameIdChooser.addUniqueFrameIds(co.getFrameIds())
//                                            //                                    }
//                                            currentEntitydata: UPNS.Entitydata {
//                                                checkout: currentCheckout
//                                                path: root.visibleEntityModel.get(index) ? root.visibleEntityModel.get(index).path : ""
//                                                onIsLoadingChanged: {
////                                                    if(isLoading) {
////                                                        // Busy indicator disabled at the moment
////                                                        entityInstantiator.loadingItems++
////                                                    } else {
////                                                        // Busy indicator disabled at the moment
////                                                        entityInstantiator.loadingItems--
////                                                    }
//                                                    if(entityInstantiator.model[index])
//                                                    {
//                                                        var idxInVisualInfoModel = entityInstantiator.model[index].idxInVisualInfoModel
//                                                        root.visualInfoModel[idxInVisualInfoModel].isLoading = isLoading
//                                                    }
//                                                }
//                                            }
//                                            onMinChanged: entityInstantiator.recalcBoundingBox()
//                                            onMaxChanged: entityInstantiator.recalcBoundingBox()
//                                            HandleBoundingBox {
//                                                layer: boundingBoxButton.checked ? gizmoLayer : invisibleLayer
//                                                min: parent.min
//                                                max: parent.max
//                                            }
//                                        }
//                                    }

//                                    Q3D.NodeInstantiator {
//                                        id: peerVisualEntityInstantiator
//                                        function recalcBoundingBox() {
//                                            boundingBoxRecalculator.start()
//                                        }
//                                        property int loadingItems: 0
//                                        property vector3d boundingboxMin // use recalc. Propertybindings would not reevaluate if model changed
//                                        property vector3d boundingboxMax // use recalc. Propertybindings would not reevaluate if model changed

//                                        //onModelChanged: recalcBoundingBox()
//                                        onObjectAdded: recalcBoundingBox()
//                                        onObjectRemoved: recalcBoundingBox()
//                                        model: root.mapitClient.state.visibleEntityInfosList
//                                        onModelChanged: {
//                                            recalcBoundingBox()
//                                            console.log("DBG: PEERMODEL CHANGED" + root.mapitClient.state.visibleEntityInfosList.count)
//                                        }

//                                        delegate: MapitEntity {
//                                            mainCameratmp: mainCamera
//                                            scene3dtmp: scene3d
//                                            coordinateSystem: coordSys
//                                            currentFrameId: root.currentFrameId

//                                            layer: pointLayer
//                                            //parametersTmp: techniqueFilter.parameters
//                                            //Currently only one checkout is supported
//                                            currentCheckout: globalApplicationState.currentCheckout
//                                            currentEntitydata: UPNS.Entitydata {
//                                                checkout: currentCheckout
//                                                path: root.mapitClient.state ? root.mapitClient.state.visibleEntityInfosList.get(index) ? root.mapitClient.state.visibleEntityInfosList.get(index).path : "" : ""
//                                            }
//                                            onMinChanged: peerVisualEntityInstantiator.recalcBoundingBox()
//                                            onMaxChanged: peerVisualEntityInstantiator.recalcBoundingBox()
//                                            HandleBoundingBox {
//                                                layer: boundingBoxButton.checked ? gizmoLayer : invisibleLayer
//                                                min: parent.min
//                                                max: parent.max
//                                            }
//                                        }
//                                    }

//                                    Q3D.Entity {
//                                        id: annotationPreviewEntity
//                                        property ObjectPicker picker: ObjectPicker {
//                                            onClicked: console.log("Clicked sphere", pick.distance, pick.triangleIndex)
//                                        }

//                                        property var meshTransform: Q3D.Transform {
//                                            matrix: appStyle.tmpPreviewMatrix
//                                        }
//                                        property var sphereMesh: SphereMesh { }
//                                        property var planeMesh: PlaneMesh { }
//                                        property var cylinderMesh: CylinderMesh { }
//                                        property var coneMesh: ConeMesh { }
//                                        property var torusMesh: TorusMesh { minorRadius: 0.3 }
//                                        property var cubeMesh: CuboidMesh { }
//                                        property var selectedMesh: appStyle.tmpPrimitiveType === "sphere" ? sphereMesh
//                                                                                                          : appStyle.tmpPrimitiveType === "plane" ? planeMesh
//                                                                                                                                                  : appStyle.tmpPrimitiveType === "cylinder" ? cylinderMesh
//                                                                                                                                                                                             : appStyle.tmpPrimitiveType === "cone" ? coneMesh
//                                                                                                                                                                                                                                    : appStyle.tmpPrimitiveType === "torus" ? torusMesh
//                                                                                                                                                                                                                                                                            : appStyle.tmpPrimitiveType === "cube" ? cubeMesh
//                                                                                                                                                                                                                                                                                                                   : sphereMesh

//                                        property var materialPhong: PhongMaterial { }
//                                        property Layer currentLayer: appStyle.tmpPlacePrimitive ? solidLayer : invisibleLayer
//                                        components: [ selectedMesh, materialPhong, meshTransform, currentLayer, picker ]
//                                    }
//                                }
//                            }
                        }
                    }
                }
                ColumnLayout {
                    spacing: 0
                    Layout.fillWidth: true
                    enabled: globalGraphBlocksEnabled
                    Layout.maximumHeight: globalGraphBlocksEnabled ? 5000 : 0
                    //Layout.minimumHeight: 120

                    StyledHeader {
                        id: nodeGraphHeader
                        Layout.fillWidth: true
                        text: "Node Graph"
                        iconSource: "image://material/ic_device_hub"
                        StyledButton {
                            isIcon: true
                            visible: !globalApplicationState.currentCheckout.isBusyExecuting
                            iconSource: "image://material/ic_play_circle_filled"
                            onClicked: gbcontrol.doOperations()
                        }
                        StyledButton {
                            isIcon: true
                            visible: globalApplicationState.currentCheckout.isBusyExecuting
                            iconSource: "image://material/ic_pause_circle_filled"
                            enabled: false
                        }
                        onCheckedChanged: if(!checked) parent.height = appStyle.controlHeightContainer
                    }
                    GraphBlocksGraphControl {
                        id: gbcontrol
                        Layout.fillHeight: true
                        Layout.fillWidth: true
                        visible: nodeGraphHeader.checked
                        Component.onCompleted: {
                            importLibrary("basic", basicLib);
                            importLibrary("internal", internalLib)
                            importLibrary("mapit", mapitLib)
                        }
                        isEditingSuperblock: false
                        sourceElement: gbcontrol
                        manualMode: true
                        function doOperations() {
                            globalOperationScheduler.operatorList = []
                            execute()
                            globalOperationScheduler.work()
                        }

                        Item {
                            id: globalOperationScheduler
                            property var operatorList: []
                            function work() {
                                operationTimer.executeOperations()
                            }
                        }
                        Timer {
                            id: operationTimer
                            interval: 100
                            repeat: false
                            onTriggered: executeOperations()
                            function executeOperations() {
                                //if(running) return
                                if(globalApplicationState.currentCheckout.isBusyExecuting) {
                                    operationTimer.start()
                                    return
                                }
                                console.log("Last Operation Status: " + globalApplicationState.currentCheckout.lastOperationStatus)
                                if(globalOperationScheduler.operatorList.length == 0) {
                                    console.log("Done executing")
                                    return
                                }
                                var op = globalOperationScheduler.operatorList.shift()
                                console.log("Starting: " + op.moduleName + ", params: " + op.parameters)
                                globalApplicationState.currentCheckout.doOperation(op.moduleName, op.parameters)
                                operationTimer.start()
                            }
                        }

                        MapitBlocksLib { id: mapitLib }
                        GraphBlocksBasicLibrary {
                            id: basicLib
                        }
                        Item {
                            id: internalLib
                            GraphBlocksSuperBlock {
                                controlManager: root
                            }
                        }
                    }
                }
            }
        }
    }
}
