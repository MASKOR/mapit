import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

import ".."

Item {
    id: root
    //// in ////
    property bool editable
    property var currentCheckout
    property string currentEntity

    function fromParameters(params) {
        parameters = params

    }

    //// out ////
    property bool valid: radiusTextfiled.valid
                      && voxelsizeTextfiled.valid
                      && entityChooser.currentEntityPath != ""
    property var parameters: {
        "radius": radiusTextfiled.text,
        "voxelsize": voxelsizeTextfiled.text,
        "target": entityChooser.currentEntityPath
    }

    //// UI ////
    ColumnLayout {
        anchors.fill: parent
        height: root.height
        RowLayout {
            Layout.fillWidth: true
            Text {
                Layout.alignment: Qt.AlignTop
                text: "Radius:"
                color: palette.text
                renderType: Text.NativeRendering
            }
            TextField {
                id: radiusTextfiled
                text:"0.01"
                //property real num: text.toFixed(8)
                property bool valid: text < validator.top && text > validator.bottom
                validator: DoubleValidator {
                    bottom: 0.0001
                    top: 1.0
                }
            }
            Text {
                Layout.alignment: Qt.AlignTop
                text: "Voxelsize:"
                color: palette.text
                renderType: Text.NativeRendering
            }
            TextField {
                id: voxelsizeTextfiled
                text:"0.004"
                //property real num: text.toFixed(8)
                property bool valid: text < validator.top && text > validator.bottom
                validator: DoubleValidator {
                    bottom: 0.0001
                    top: 1.0
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
            EntityChooser {
                id: entityChooser
                Layout.fillWidth: true
                Layout.fillHeight: true
                currentCheckout: root.currentCheckout
                currentEntityPath: root.currentEntity
            }
        }
        SystemPalette {
            id: palette
        }
    }
}
