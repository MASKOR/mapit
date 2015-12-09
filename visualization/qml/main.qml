import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1

import fhac.upns 1.0;

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

    MapManager {
        id: mapMan
        Component.onCompleted: {
            var allTheMaps = mapMan.listMaps();
            allTheMaps.forEach(function(entry) {
                maps.appendMap(entry, false, mapMan);
            });
        }
    }
    ListModel {
        id:maps
        property bool lazy: true
        property var idToIndex
        Component.onCompleted: {
            idToIndex = {}
        }
        function appendMap(id, select, mman) {
            if(idToIndex[id] !== undefined) {
                console.log("map was added twice")
                return
            }
//            for(var i=0 ; maps.count ; ++i) {
//                if(maps.get(i).mapId === id) {
//                    console.log("map was added twice");
//                    return;
//                }
//            }
            maps.append({mapId:id})
            if(select)
            {
                mapsList.currentIndex = maps.count-1
            }
            if(mman) {
                mman.getMap(id)
            }
        }
    }

    RowLayout {
        anchors.fill: parent
        ColumnLayout {
            width: 220
            Layout.fillHeight: true
            Button {
                width: Layout.width
                text: "load_pointcloud"
                onClicked: {
                    dialog_load_pointcloud.open();
                }
                Dialog {
                    id: dialog_load_pointcloud
                    height: 30
                    standardButtons: StandardButton.Ok | StandardButton.Cancel
                    onAccepted: {
                        var opdesc = {
                            "operatorname":"load_pointcloud",
                            "params": [
                                {
                                    "key": "filename",
                                    //"strval": "/home/dbulla/Pointclouds/scene exports/FH-DGeb-Hoersaal-linksoben.pcd"
                                    "strval": fileNamePcd.text
                                }
                            ]
                        }
                        var result = mapMan.doOperation( opdesc );
                        drawingArea.mapId = result.output.params.target.mapval;
                        maps.appendMap(result.output.params.target.mapval, true);
                    }
                    RowLayout {
                        anchors.fill: parent
                        TextField {
                            id:fileNamePcd
                        }
                        Button {
                            text: "Open"
                            onClicked: {
                                openPcdFileDialog.open()
                            }
                        }
                    }
                    FileDialog {
                        id: openPcdFileDialog
                        title: "Open Pcd"
                        selectExisting: true
                        selectFolder: false
                        selectMultiple: false
                        onAccepted: {
                            var filename = fileUrl.toString()
                            filename = filename.replace(/^(file:\/{2})/,"")
                            fileNamePcd.text = filename
                        }
                    }
                }
            }
            ListView {
                id: mapsList
                Layout.fillHeight: true
                model: maps
                delegate: Text {
                    text: mapId
                    color: mapsList.currentIndex == index?"blue":"black"
                    MouseArea {
                        anchors.fill: parent
                        onClicked: mapsList.currentIndex = index
                    }
                }
                onCurrentIndexChanged: {
                    drawingArea.mapId = maps.get(currentIndex).mapId;
                }
            }
        }
        MapsRenderViewport {
            id: drawingArea
            Layout.fillWidth: true
            Layout.fillHeight: true
            mapManager: mapMan
        }
    }
}
