import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1

import fhac.upns 1.0
import "../components"
import "."

ButtonDefaultOperation {
    id: root
    text: "Voxelgridfilter"
    property var inputMapId
    property var outputMapId
    onBuildOperationDescription: {
        operationDescription = {
            "operatorname":"voxelgridfilter",
            "params": [
                {
                    "key": "target",
                    "mapval": mapChooser.choosenMapId
                }
            ]
        }
    }
    onInitializeDialog: {
        mapChooser.choosenMapId = inputMapId
    }
    onOperationFinished: {
        outputMapId = result.output.params.target.mapval
    }
    ColumnLayout {
        id: controlsLayout
        //anchors.fill: parent // dialog adapts it's size
        height: 200
        RowLayout {
            Layout.fillWidth: true
            Text {
                Layout.alignment: Qt.AlignTop
                text: "Leafsize:"
                color: palette.text
                renderType: Text.NativeRendering
            }
            TextField {
                id:fileNamePcd
                validator: DoubleValidator {}
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
        SystemPalette {
            id: palette
        }
    }
}

//Button {
//    id: root
//    property var inputMapId
//    property var outputMapId
//    signal finishedOperation
//    text: "Voxelgridfilter"
//    onClicked: {
//        settingsDialog.open();
//    }
//    Dialog {
//        id: settingsDialog
//        height: 30
//        standardButtons: StandardButton.Ok | StandardButton.Cancel
//        onVisibleChanged: {
//            if(visible) {
//                mapChooser.choosenMapId = root.presetMapId
//            }
//        }
//        onAccepted: {
//            var opdesc = {
//                "operatorname":"voxelgridfilter",
//                "params": [
//                    {
//                        "key": "target",
//                        "mapval": mapChooser.choosenMapId
//                    }
//                ]
//            }

//            var result = Globals.doOperation( opdesc, function(result) {
//                if(result.status !== 0) {
//                    var errMsg = "An operation returned an error.\n";
//                    errMsg += "See the log file for more information.\n";
//                    errMsg += "\n"
//                    errMsg += "(Code: "+result.status+")"
//                    errorDialog.showError(errMsg);
//                    return
//                }
//                root.outputMapId = result.output.params.target.mapval
//                root.finishedOperation( root.outputMapId )
//            });
//        }
//        ColumnLayout {
//            //anchors.fill: parent // dialog adapts it's size to children
//            height: 200
//            RowLayout {
//                Layout.fillWidth: true
//                Text {
//                    Layout.alignment: Qt.AlignTop
//                    text: "Leafsize:"
//                    color: palette.text
//                    renderType: Text.NativeRendering
//                }
//                TextField {
//                    id:fileNamePcd
//                    validator: DoubleValidator {}
//                }
//            }
//            RowLayout {
//                Layout.fillWidth: true
//                Layout.fillHeight: true
//                Text {
//                    Layout.alignment: Qt.AlignTop
//                    text: "Target:"
//                    color: palette.text
//                    renderType: Text.NativeRendering
//                }
//                MapChooser {
//                    id: mapChooser
//                    Layout.fillWidth: true
//                    Layout.fillHeight: true
//                }
//            }
//        }
//    }
//    SystemPalette {
//        id: palette
//    }
//}
