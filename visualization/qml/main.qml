import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1

import fhac.upns 1.0
import "components"

ApplicationWindow {
    title: qsTr("Map Visualization")
    width: 640
    height: 480
    visible: true
    SystemPalette {
        id: palette
    }
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
    Dialog {
        id: errorDialog
        property alias text: errorText.text
        title: "Error"
        standardButtons: StandardButton.Ok
        visible: false
        Text {
            id: errorText
            renderType: Text.NativeRendering
            color: palette.text
        }
        function showError(msg) {
            text = msg
            visible = true
        }
    }

    SplitView {
        anchors.fill: parent
        orientation: Qt.Horizontal
        ColumnLayout {
            width: 220
            Layout.minimumWidth: 100
            Layout.maximumWidth: Number.MAX_VALUE
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
                    onVisibleChanged: {
                        if(visible) {
                            mapChooser.choosenMapId = Globals.mapIdsModel.get(mapsList.currentIndex).mapId
                        }
                    }
                    onAccepted: {
                        var opdesc = {
                            "operatorname":"load_pointcloud",
                            "params": [
                                {
                                    "key": "filename",
                                    "strval": fileNamePcd.text
                                }
                            ]
                        }
                        if(mapChooser.choosenMapId !== "" && parseInt(mapChooser.choosenMapId) !== 0) {
                            opdesc.params.push(
                            {
                                "key": "target",
                                "mapval": mapChooser.choosenMapId
                            });
                            console.log("dbg: using old" + mapChooser.choosenMapId);
                        } else if(mapChooser.choosenMapName !== "") {
                            opdesc.params.push(
                            {
                                "key": "mapname",
                                "strval": mapChooser.choosenMapName
                            });
                            console.log("dbg: creating nw" + mapChooser.choosenMapName);
                        } else {
                            console.log("dbg: none of the two");
                        }
                        var result = Globals.doOperation( opdesc, function(result) {
                            if(result.status !== 0) {
                                var errMsg = "An operation returned an error.\n";
                                errMsg += "See the log file for more information.\n";
                                errMsg += "\n"
                                errMsg += "(Code: "+result.status+")"
                                errorDialog.showError(errMsg);
                                return
                            }
                            var loadedMapId = result.output.params.target.mapval
                            mapsList.currentIndex = Globals.mapIdsModel.indexOfMap( loadedMapId )
                        });
                    }
                    ColumnLayout {
                        //anchors.fill: parent // dialog adapts it's size
                        height: 200
                        RowLayout {
                            Layout.fillWidth: true
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
                        RowLayout {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            Text {
                                Layout.alignment: Qt.AlignTop
                                text: "Target:"
                                color: palette.text
                                renderType: Text.NativeRendering
                            }
                            MapChooser {
                                id: mapChooser
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                            }
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
            ListView {
                id: mapsList
                Layout.fillHeight: true
                Layout.fillWidth: true
                clip: true
                model: Globals.mapIdsModel
                highlight: Rectangle {
                    width: mapsList.currentItem.width + 2
                    height: mapsList.currentItem.height
                    color: palette.highlight
                    radius: 1
                    y: mapsList.currentItem.y
                }
                highlightFollowsCurrentItem: false
                delegate: Text {
                    renderType: Text.NativeRendering
                    text: Globals.getMap(mapId).name
                    onTextChanged: {
                        if(text === "") text = "<empty name>"
                    }
                    color: mapsList.currentIndex == index?palette.highlightedText:palette.text
                    MouseArea {
                        anchors.fill: parent
                        onClicked: mapsList.currentIndex = index
                    }
                }

                onCurrentIndexChanged: {
                    // simply using currentItem.mapId does not work
                    drawingArea.mapId = Globals.mapIdsModel.get(currentIndex).mapId;
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
}
