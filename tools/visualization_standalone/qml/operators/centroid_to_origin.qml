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
        checkboxAABB.checked = params.useAABB
        genTfCheckbox.checked = params.genTf
    }

    //// out ////
    property bool valid: entityChooser.valid
    property var parameters: {
        "target": entityChooser.currentEntityPath,
        "useAABB": checkboxAABB.checked,
        "genTf": genTfCheckbox.checked
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
            text: "Bounding Box:"
        }
        StyledCheckBox {
            id: checkboxAABB
        }
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "Gen Transform:"
        }
        StyledCheckBox {
            id: genTfCheckbox
        }
    }
}
