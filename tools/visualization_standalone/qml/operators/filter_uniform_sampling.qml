import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

import ".."

ColumnLayout {
    id: root
    //// in ////
    property bool editable
    property var currentCheckout
    property string currentEntityPath

    function fromParameters(params) {
        sourceEntityChooser.currentEntityPath = params.source;
        targetEntityChooser.currentEntityPath = params.target;
        radiusInput.text = params.radius;
    }

    //// out ////
    property bool valid: sourceEntityChooser.currentEntityPath != ""
                         && targetEntityChooser.currentEntityPath != ""
                         && parseFloat(radiusInput.text) >= 0.0
    property var parameters: {
        "source": sourceEntityChooser.currentEntityPath,
                "target": targetEntityChooser.currentEntityPath,
                "radius": parseFloat(radiusInput.text)
    }

    //// UI ////
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
            currentEntityPath: sourceEntityChooser.currentEntityPath + "_unismp_r" + radiusInput.text
        }
    }

    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "radius"
        }
        StyledTextField {
            id: radiusInput
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "0.01"
        }
    }
}
