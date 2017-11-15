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
        frameIdInput.text = params.frame_id
        //secInput.text = params.stamp.sec
        //nsecInput.text = params.stamp.nsec
    }

    //// out ////
    property bool valid: entityChooser.valid
    property var parameters: {
        "target": entityChooser.currentEntityPath,
        "frame_id": frameIdInput.text,
        "stamp": {
            "sec": parseInt(secInput.text),
            "nsec": parseInt(nsecInput.text)
        }
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
            text: "frame_id"
        }
        StyledTextField {
            id: frameIdInput
            Layout.fillWidth: true
        }
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "Seconds: "
        }
        StyledTextField {
            id: secInput
            Layout.fillWidth: true
            validator: IntValidator {}
            text: "0"
        }
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "Nanoseconds: "
        }
        StyledTextField {
            id: nsecInput
            Layout.fillWidth: true
            validator: IntValidator {}
            text: "0"
        }
    }
}
