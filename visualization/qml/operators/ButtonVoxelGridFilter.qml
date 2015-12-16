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
                }, {
                    "key": "leafsize",
                    "realval": leafSizeText.text
                }
            ]
        }
        console.log("opdesc: " + operationDescription.params[1].realval + " t: " + leafSizeText.text)
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
        height: 400
        RowLayout {
            Layout.fillWidth: true
            Text {
                Layout.alignment: Qt.AlignTop
                text: "Leafsize:"
                color: palette.text
                renderType: Text.NativeRendering
            }
            TextField {
                id: leafSizeText
                validator: DoubleValidator {}
                text: "0.01"
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
