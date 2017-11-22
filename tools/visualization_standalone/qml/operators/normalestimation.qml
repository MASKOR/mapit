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
        entityChooser.currentEntityPath = params.target
    }

    //// out ////
    property bool valid: entityChooser.valid
    property var parameters: {
        "target":entityChooser.currentEntityPath,
        "radius": radiusCb.checked ? parseFloat(radiusInput.text): "",
        "k": kInput.checked ? parseInt(kInput.text) : ""
    }

    //// UI ////
    HelperTarget {
        Layout.fillWidth: true
        id: entityChooser
        currentEntityPath: root.currentEntityPath
        dialogRoot: root
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "Radius"
        }
        StyledTextField {
            id: radiusInput
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "0.1"
            onFocusChanged: if(focus) radiusCb.checked = true
        }
        StyledCheckBox {
            id: radiusCb
            onCheckedChanged: if(checked) kCb.checked = false
        }
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "k"
        }
        StyledTextField {
            id: kInput
            Layout.fillWidth: true
            validator: IntValidator {}
            text: "5"
            onFocusChanged: if(focus) kCb.checked = true
        }
        StyledCheckBox {
            id: kCb
            checked: true
            onCheckedChanged: if(checked) radiusCb.checked = false
        }
    }
}
