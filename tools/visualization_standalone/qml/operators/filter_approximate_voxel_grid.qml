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
    property string currentEntityPath

    function fromParameters(params) {
        sourceEntityChooser.currentEntityPath = params.source;
        targetEntityChooser.currentEntityPath = params.target;
        leafSizeInput.text = params.leafsize;
        downsampleAllDataInput.on = params.downsampleAllData;
    }

    //// out ////
    property bool valid: sourceEntityChooser.currentEntityPath != ""
                         && targetEntityChooser.currentEntityPath != ""
                         && leafSizeInput.text != ""
    property var parameters: {
        "source": sourceEntityChooser.currentEntityPath,
        "target": targetEntityChooser.currentEntityPath,
        "leafSize": parseFloat(leafSizeInput.text),
        "downsampleAllData": downsampleAllDataInput.checked
    }

    //// UI ////
    ColumnLayout {
        anchors.fill: parent
        height: root.height

        RowLayout {
            Layout.fillWidth: true
            z: 20
            StyledLabel {
                text: "Source:"
            }
            EntityChooser {
                id: sourceEntityChooser
                Layout.fillWidth: true
                currentCheckout: root.currentCheckout
                currentEntityPath: root.currentEntityPath
            }
        }

        RowLayout {
            Layout.fillWidth: true
            z: 10
            StyledLabel {
                text: "Target:"
            }
            EntityChooser {
                id: targetEntityChooser
                Layout.fillWidth: true
                currentCheckout: root.currentCheckout
                currentEntityPath: sourceEntityChooser.currentEntityPath + "_avxg"
                                   + "_s" + leafSizeInput.text
            }
        }

        RowLayout {
            Layout.fillWidth: true
            StyledLabel {
                text: "leaf size"
            }
            StyledTextField {
                id: leafSizeInput
                Layout.fillWidth: true
                validator: DoubleValidator {}
                text: "0.01"
            }
        }

        RowLayout {
            Layout.fillWidth: true
            StyledLabel {
                text: "downsample all data"
            }
            Switch {
                Layout.fillWidth: true
                id: downsampleAllDataInput
                checked: true
            }
        }

        Item {
            Layout.fillHeight: true
        }
    }
}
