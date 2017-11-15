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
        entityChooser.currentEntityPath = params.input
    }

    //// out ////
    property bool valid: entityChooser.valid && detailTextfield.valid
    property var parameters: {
        "input" : entityChooser.currentEntityPath,
        "output" : entityChooser.currentEntityPath + "-ply", //TODO: because input != output at the moment (if entitytypes differ and both streams are open at the same time)
        "detail" : parseFloat(detailTextfield.text)
    }

    //// UI ////
    StyledLabel {
        Layout.fillWidth: true
        text: "Detail:"
    }
    StyledTextField {
        Layout.fillWidth: true
        id: detailTextfield
        text:"0.1"
        //property real num: text.toFixed(8)
        property bool valid: text <= validator.top && text >= validator.bottom
        validator: DoubleValidator {
            bottom: 0.0
            top: 1.0
        }
    }
    HelperTarget {
        id: entityChooser
        currentEntityPath: root.currentEntityPath
        dialogRoot: root
    }
}
