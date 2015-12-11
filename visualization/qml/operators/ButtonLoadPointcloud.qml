import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1

import fhac.upns 1.0
import "../components"

Button {
    property var inputMapId
    property var outputMapId
    text: "Load PCD"
    onClicked: {
        settingsDialog.open();
    }
    Dialog {
        id: settingsDialog
        height: 30
        standardButtons: StandardButton.Ok | StandardButton.Cancel
        onVisibleChanged: {
            if(visible) {
                mapChooser.choosenMapId = root.inputMapId
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
