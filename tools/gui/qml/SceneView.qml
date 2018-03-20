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

import fhac.mapit 1.0 as Mapit

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
    property var currentWorkspace: globalApplicationState.currentWorkspace
    property var currentEntitydataTransform: globalApplicationState.currentEntityTransform
    property ListModel visibleEntityModel//: entityInstantiator.model
    property var allVisualInfoModel
    property alias camera: mainCamera
    property alias currentFrameId: frameIdChooser.currentText
    property MapitClient mapitClient: globalApplicationState.mapitClient

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
                    //currentWorkspace: root.currentWorkspace
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
                visible: false // hide primitive icons for now
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

                    Mapit.Raycast {
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
                            Mapit.Raycast {
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
                                currentWorkspace: globalApplicationState.currentWorkspace
                                mapitClient: root.mapitClient
                                camera: mainCamera
                                appStyleOptional: appStyle

                                gridLayer: gridLayer
                                gizmoLayer: gizmoLayer
                                invisibleLayer: invisibleLayer
                                pointLayer: pointLayer
                                solidLayer: solidLayer
                            }
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
                            visible: !globalApplicationState.currentWorkspace.isBusyExecuting
                            iconSource: "image://material/ic_play_circle_filled"
                            onClicked: gbcontrol.doOperations()
                        }
                        StyledButton {
                            isIcon: true
                            visible: globalApplicationState.currentWorkspace.isBusyExecuting
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
                                if(globalApplicationState.currentWorkspace.isBusyExecuting) {
                                    operationTimer.start()
                                    return
                                }
                                console.log("Last Operation Status: " + globalApplicationState.currentWorkspace.lastOperationStatus)
                                if(globalOperationScheduler.operatorList.length == 0) {
                                    console.log("Done executing")
                                    return
                                }
                                var op = globalOperationScheduler.operatorList.shift()
                                console.log("Starting: " + op.moduleName + ", params: " + op.parameters)
                                globalApplicationState.currentWorkspace.doOperation(op.moduleName, op.parameters)
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
