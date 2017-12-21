import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

import ".."
import "../components"

ColumnLayout {
    id: root
    //// in ////
    property bool editable
    property string currentEntityPath

    function fromParameters(params) {
        fileChooser.filename = params.filename
        entityChooser.currentEntityPath = params.target
    }

    //// out ////
    property bool valid:    fileChooser.valid
                         && entityChooser.valid
    property var parameters: {
        "filename": fileChooser.filename,
        "target": entityChooser.currentEntityPath,
        "frame_id": frameId.currentText,
        "sec":      stampSec.text,
        "nsec:":    stampNSec.text
    }

    //// UI ////
    HelperOpenFile {
        Layout.fillWidth: true
        id: fileChooser
        fileExtension: "pcd"
    }
    HelperTarget {
        Layout.fillWidth: true
        id: entityChooser
        currentEntityPath: root.currentEntityPath
    }
    RowLayout {
        StyledLabel {
            text: "frame_id:"
        }
        FrameIdChooser {
            Layout.fillWidth: true
            Layout.minimumWidth: 100
            id: frameId
            allowNew: true
            currentCheckout: root.currentCheckout
        }
    }
    RowLayout {
        StyledLabel {
            text: "Stamp:"
        }
        StyledTextField {
            id: stampSec
            validator: IntValidator {}
            placeholderText: "sec"
        }
        StyledTextField {
            Layout.fillWidth: true
            id: stampNSec
            validator: IntValidator {}
            placeholderText: "nsec"
        }
    }
}
