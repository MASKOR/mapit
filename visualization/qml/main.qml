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
            Layout.minimumWidth: 100
            Layout.maximumWidth: Number.MAX_VALUE
            Layout.fillHeight: true
            anchors.fill: parent //TODO: proper resizing
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
                    inputMapId: Globals.mapIdsModel.get(mapsList.currentIndex).mapId
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
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumWidth: 50
            mapManager: Globals._mapManager
            mapId: mapsList.currentMapId
            layerId: mapLayers.currentLayerId
            property real angleX: 0
            property real angleY: 0
            property real zoom: 1.0
            Behavior on angleX {
                SmoothedAnimation { velocity: 10.0; reversingMode: SmoothedAnimation.Immediate }
            }
            Behavior on angleY {
                SmoothedAnimation { velocity: 10.0; reversingMode: SmoothedAnimation.Immediate }
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
            property var zoomMat: Qt.matrix4x4(
                                   zoom, 0.0,  0.0, 0.0,
                                   0.0,  zoom, 0.0, 0.0,
                                   0.0,  0.0, zoom, 0.0,
                                   0.0,  0.0,  0.0, 1.0)
            matrix: rotY.times(rotX).times(zoomMat)
            MouseArea {
                id: screenMouse
                anchors.fill: parent
                hoverEnabled: true
                property real mx
                property real my
                property real ax
                property real ay
                onWheel: {
                    drawingArea.zoom += wheel.angleDelta.y*0.01
                }
                onPressed: {
                    mx = mouseX
                    my = mouseY
                    ax = drawingArea.angleX
                    ay = drawingArea.angleY
                }
                onPositionChanged: {
                    if(!pressed) return
                    drawingArea.angleX = ax + (mx-mouseX)*0.05
                    drawingArea.angleY = ay + (my-mouseY)*0.05
                }
            }
        }
    }
    SystemPalette {
        id: palette
    }
}
