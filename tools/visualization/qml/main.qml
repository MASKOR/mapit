import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1

import fhac.upns 1.0
import "components"
import "operators"

ApplicationWindow {
    title: qsTr("Map Visualization")
    width: 640
    height: 480
    visible: true
    menuBar: MenuBar {
        Menu {
            title: qsTr("&File")
            MenuItem {
                text: qsTr("&Open Project")
                onTriggered: openFileDialog.open()
            }
            MenuItem {
                text: qsTr("&Commit")
                onTriggered: saveFileDialog.open()
            }
            MenuItem {
                text: qsTr("Save Project &As")
                onTriggered: saveFileDialog.open()
            }
            MenuItem {
                text: qsTr("E&xit")
                onTriggered: Qt.quit();
            }
        }
        Menu {
            title: qsTr("&Edit")
            MenuItem {
                text: qsTr("&Copy")
                onTriggered: console.log("not yet implemented.");
            }
            MenuItem {
                text: qsTr("&Paste")
                onTriggered: {

                }
            }
        }
        Menu {
            title: qsTr("&View")
            MenuItem {
                id: detailHiItem
                text: qsTr("&Hi Detail")
                checkable: true
                checked: true
                onCheckedChanged: {
                }
            }
            MenuItem {
                id: detailMidtem
                text: qsTr("&Mid Detail")
                checkable: true
                checked: false
                onCheckedChanged: {
                }
            }
            MenuItem {
                id: detailLoItem
                text: qsTr("&Low Detail")
                checkable: true
                checked: false
                onCheckedChanged: {
                }
            }
            MenuSeparator { }
            MenuItem {
                id: executeItem
                text: qsTr("Lower Detail when &moving")
                enabled: false
                onTriggered: {
                }
            }
            MenuSeparator { }
            MenuItem {
                id: showCenter
                text: qsTr("Show Center &Cross")
                checkable: true
                checked: true
            }
        }
        Menu {
            title: qsTr("&Window")
            MenuItem {
                text: qsTr("Show &Operations Pane")
                checkable: true
                checked: true
                onCheckedChanged: {
                }
            }
            MenuItem {
                text: qsTr("Show &Maps/Layers Pane")
                checkable: true
                checked: false
                onCheckedChanged: {
                }
            }
        }
    }

    SplitView {
        anchors.fill: parent
        orientation: Qt.Horizontal

        Item {
            width: 300
            Layout.minimumWidth: 100
            Layout.maximumWidth: Number.MAX_VALUE
            //Layout.fillHeight: true
            //anchors.fill: parent //TODO: proper resizing
            ColumnLayout {
                anchors.fill: parent
                ButtonLoadPointcloud {
                    Layout.minimumWidth: 100
                    Layout.fillWidth: true
                    inputMapId: Globals.mapIdsModel.get(mapsList.currentIndex).mapId
                    onOutputMapIdChanged: {
                        mapsList.currentMapId = outputMapId
                        Globals.reload(true)
                        drawingArea.reload()
                    }
                }
                ButtonVoxelGridFilter {
                    Layout.minimumWidth: 100
                    Layout.fillWidth: true
                    inputMapId: mapsList.currentMapId
                    onOutputMapIdChanged: {
                        mapsList.currentMapId = outputMapId
                        Globals.reload(true)
                        drawingArea.reload()
                    }
                }
                MapsListView {
                    id: mapsList
                    Layout.fillHeight: true
                    Layout.minimumWidth: 100
                    Layout.fillWidth: true
//                    Rectangle {
//                        anchors.fill: parent
//                        border.width:  2
//                        border.color: "red"
//                    }
                }
                Text {
                    text: "Layers"
                }
                MapLayerView {
                    id: mapLayers
                    Layout.fillWidth: true
                    Layout.minimumWidth: 100
                    Layout.preferredHeight: 200
                    mapId: mapsList.currentMapId
                }
            }
        }
        MapsRenderViewport {
            id: drawingArea
            Layout.minimumWidth: 50
            Layout.fillWidth: true
            //Layout.fillHeight: true
            mapManager: Globals._mapManager
            mapId: mapsList.currentMapId
            layerId: mapLayers.currentLayerId
            property var pos: Qt.vector3d(0.0,0.0,0.0)
            property var offsX: 0.0
            property var offsY: 0.0
            property real angleX: 0
            property real angleY: 0
            property real zoom: 1.0
            Behavior on angleX {
                SmoothedAnimation { velocity: 10.0; reversingMode: SmoothedAnimation.Immediate }
            }
            Behavior on angleY {
                SmoothedAnimation { velocity: 10.0; reversingMode: SmoothedAnimation.Immediate }
            }
            Rectangle {
                x: parent.width/2
                y: parent.height/2-5
                width: 1
                height: 11
                color: "white"
                visible: showCenter.checked
            }
            Rectangle {
                x: parent.width/2-5
                y: parent.height/2
                width: 11
                height: 1
                color: "white"
                visible: showCenter.checked
            }

            property var rotX: Qt.matrix4x4(
                                   Math.cos(angleX), 0.0, -Math.sin(angleX), 0.0,
                                   0.0,              1.0, 0.0,               0.0,
                                   Math.sin(angleX), 0.0, Math.cos(angleX),  0.0,
                                   0.0,              0.0, 0.0,               1.0)
            property var rotY: Qt.matrix4x4(
                                   1.0,              0.0, 0.0,               0.0,
                                   0.0, Math.cos(angleY), -Math.sin(angleY), 0.0,
                                   0.0, Math.sin(angleY),  Math.cos(angleY), 0.0,
                                   0.0,              0.0, 0.0              , 1.0)
            // center of rotation
            property var posMat: Qt.matrix4x4(
                                     1.0, 0.0, 0.0, pos.x,
                                     0.0, 1.0, 0.0, pos.y,
                                     0.0, 0.0, 1.0, pos.z,
                                     0.0, 0.0, 0.0, 1.0)
            // offset while translating
            property var offsMat: Qt.matrix4x4(
                                     1.0, 0.0, 0.0, offsX,
                                     0.0, 1.0, 0.0, offsY,
                                     0.0, 0.0, 1.0, 0.0,
                                     0.0, 0.0, 0.0, 1.0)
            property var zoomMat: Qt.matrix4x4(
                                   zoom, 0.0,  0.0, 0.0,
                                   0.0,  zoom, 0.0, 0.0,
                                   0.0,  0.0, zoom, 0.0,
                                   0.0,  0.0,  0.0, 1.0)
            matrix: offsMat.times(rotY).times(rotX).times(posMat).times(zoomMat)
            MouseArea {
                id: screenMouse
                anchors.fill: parent
                hoverEnabled: true
                acceptedButtons: Qt.LeftButton | Qt.MiddleButton | Qt.RightButton
                property bool rotating: false
                property bool translating: false
                property real mx
                property real my
                property real ax
                property real ay
                onWheel: {
                    drawingArea.zoom *= Math.pow(2.0, wheel.angleDelta.y*0.01)
                }
                function pressedButtonsChanged() {
                    var leftButton = screenMouse.pressedButtons & Qt.LeftButton
                    var rightButton = screenMouse.pressedButtons & Qt.RightButton
                    var middleButton = screenMouse.pressedButtons & Qt.MiddleButton
                    if(leftButton && !rightButton) {
                        rotating = true
                        translating = false
                    } else if(middleButton || leftButton && rightButton) {
                        rotating = false
                        translating = true
                    } else {
                        rotating = false
                        translating = false
                    }
                }
                onPressed: {
                    mx = mouseX
                    my = mouseY
                    ax = drawingArea.angleX
                    ay = drawingArea.angleY
                    pressedButtonsChanged()
                }
                onReleased: {
                    pressedButtonsChanged()
                }
                onTranslatingChanged: {
                    if(!translating) {
                        if(drawingArea.offsX !== 0.0 || drawingArea.offsY !== 0.0) {
                            var finalRotationMatrix = drawingArea.rotY.times(drawingArea.rotX)
                            var posV4 = finalRotationMatrix.inverted().times(Qt.vector4d(drawingArea.offsX,
                                                                              drawingArea.offsY,
                                                                                            0.0,
                                                                                            1.0))
                            drawingArea.pos = drawingArea.pos.plus(Qt.vector3d(posV4.x, posV4.y, posV4.z))

                            drawingArea.offsX = 0.0
                            drawingArea.offsY = 0.0
                        }
                    }
                }
                onPositionChanged: {
                    var movementx = (mx - mouseX) * 0.05
                    var movementy = (my - mouseY) * 0.05
                    if(rotating) {
                        drawingArea.angleX = ax + movementx
                        drawingArea.angleY = ay + movementy
                    } else if(translating) {
                        drawingArea.offsX = movementx*-0.1
                        drawingArea.offsY = movementy*-0.1
                    }
                }
            }
        }
    }
    SystemPalette {
        id: palette
    }
}
