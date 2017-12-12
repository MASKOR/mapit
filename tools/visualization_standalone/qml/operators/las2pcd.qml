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
        entityChooser.currentEntityPath = params.target
        demeanCheckbox.checked = params.demean;
        normalizeCheckbox.checked = params.normalize;
    }

    //// out ////
    property bool valid: entityChooser.valid
    property var parameters: {
        "target":entityChooser.currentEntityPath,
        "demean": demeanCheckbox.checked,
        "normalize": normalizeCheckbox.checked,
        "normalizeScale": parseFloat("1.0") //TODO
    }

    //// UI ////
    HelperTarget {
        Layout.fillWidth: true
        id: entityChooser
        currentEntityPath: root.currentEntityPath
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "Demean:"
        }
        CheckBox {
            id: demeanCheckbox
            Layout.fillWidth: true
        }
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "Normalize"
        }
        CheckBox {
            id: normalizeCheckbox
            Layout.fillWidth: true
        }
    }
    SystemPalette {
        id: palette
    }
}
