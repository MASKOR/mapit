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
                    "mapval": mapChooser.choosenMapId,
                    "layerval": layerChooser.choosenLayerId
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
        width: 500
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
                text: "Target: Map:"
                color: palette.text
                renderType: Text.NativeRendering
                Layout.fillHeight: true
            }
            MapChooser {
                id: mapChooser
                Layout.fillHeight: true
                width: 100
            }
            Text {
                Layout.alignment: Qt.AlignTop
                text: "Layer:"
                color: palette.text
                renderType: Text.NativeRendering
                Layout.fillHeight: true
            }
            LayerChooser {
                id: layerChooser
                Layout.fillHeight: true
                width: 100
                rootMap: Globals.getMap(mapChooser.choosenMapId)
                allowNewLayer: false
            }
        }
        SystemPalette {
            id: palette
        }
    }
}
