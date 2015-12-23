import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1

import fhac.upns 1.0
import "../components"



ButtonDefaultOperation {
    id: root
    text: "Load PCD"
    property var inputMapId
    property var outputMapId
    onBuildOperationDescription: {


        operationDescription = {
            "operatorname":"load_pointcloud",
            "params": [
                {
                    "key": "filename",
                    "strval": fileNamePcd.text
                }
            ]
        }
        if(mapChooser.choosenMapId !== "" && parseInt(mapChooser.choosenMapId) !== 0) {
            operationDescription.params.push(
            {
                "key": "target",
                "mapval": mapChooser.choosenMapId
            });
        } else if(mapChooser.choosenMapName !== "") {
            operationDescription.params.push(
            {
                "key": "mapname",
                "strval": mapChooser.choosenMapName
            });
        } else {
            // new unnamed map will be created
        }
    }
    onInitializeDialog: {
        mapChooser.choosenMapId = root.inputMapId
    }
    onOperationFinished: {
        outputMapId = result.output.params.target.mapval
    }
    ColumnLayout {
        //anchors.fill: parent // dialog adapts it's size
        height: 400
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
        SystemPalette {
            id: palette
        }
    }
}
