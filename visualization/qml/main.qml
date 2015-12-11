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
        ColumnLayout {
            Layout.minimumWidth: 100
            Layout.maximumWidth: Number.MAX_VALUE
            Layout.fillHeight: true
            ButtonLoadPointcloud {
                Layout.minimumWidth: 100
                Layout.fillWidth: true
                inputMapId: Globals.mapIdsModel.get(mapsList.currentIndex).mapId
                onOutputMapIdChanged: {
                    mapsList.currentIndex = Globals.mapIdsModel.indexOfMap( outputMapId )
                }
            }
            ButtonVoxelGridFilter {
                Layout.minimumWidth: 100
                Layout.fillWidth: true
                inputMapId: Globals.mapIdsModel.get(mapsList.currentIndex).mapId
                onOutputMapIdChanged: {
                    mapsList.currentIndex = Globals.mapIdsModel.indexOfMap( outputMapId )
                }
            }
            MapsListView {
                id: mapsList
                Layout.fillHeight: true
                Layout.minimumWidth: 100
                Layout.fillWidth: true
                onCurrentIndexChanged: {
                    // simply using currentItem.mapId does not work
                    var newMapId = Globals.mapIdsModel.get(currentIndex).mapId
                    drawingArea.mapId = newMapId
                    mapLayers.mapId = newMapId
                }
            }
            MapLayerView {
                id: mapLayers
                Layout.fillWidth: true
                Layout.minimumWidth: 100
                Layout.preferredHeight: 200
                onCurrentIndexChanged: {
                }
            }
        }
        MapsRenderViewport {
            id: drawingArea
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumWidth: 50
            mapManager: Globals._mapManager
        }
    }
    SystemPalette {
        id: palette
    }
}
