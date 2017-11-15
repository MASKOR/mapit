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
        leafsizeInput.text = params.leafsize
    }

    //// out ////
    property bool valid: entityChooser.valid && leafsizeInput.text != ""
    property var parameters: {
        "target": entityChooser.currentEntityPath,
        "leafsize": parseFloat(leafsizeInput.text)
    }

    //// UI ////
    HelperTarget {
        id: entityChooser
        currentEntityPath: root.currentEntityPath
        dialogRoot: root
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "Leafsize"
        }
        StyledTextField {
            id: leafsizeInput
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "0.01"
            onTextChanged: console.log(parameters.leafsize)
        }
    }
}
