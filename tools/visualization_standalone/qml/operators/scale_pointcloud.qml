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

    //// out ////
    property bool valid:   true
    property var parameters: {
                "target"   : entityChooserTarget.currentEntityPath,
                "factor-x" : parseFloat(factorX.text),
                "factor-y" : parseFloat(factorY.text),
                "factor-z" : parseFloat(factorZ.text)
    }

    //// UI ////
    GridLayout {
        z: 100
        // target
        StyledLabel {
            Layout.column: 0
            Layout.row: 0
            text: "Target"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 0
            z: 110
            id: entityChooserTarget
            Layout.fillWidth: true
            currentEntityPath: root.currentEntityPath
        }

        // min/max values
        StyledLabel {
            Layout.column: 0
            Layout.row: 1
            text: "min:"
        }
        StyledLabel {
            Layout.column: 1
            Layout.row: 1
            text: "max:"
        }
        // x
        StyledLabel {
            Layout.column: 0
            Layout.row: 2
            text: "factor-x:"
        }
        StyledTextField {
            Layout.column: 1
            Layout.row: 2
            id: factorX
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "1.0"
        }
        // y
        StyledLabel {
            Layout.column: 0
            Layout.row: 3
            text: "factor-y"
        }
        StyledTextField {
            Layout.column: 1
            Layout.row: 3
            id: factorY
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "1.0"
        }
        // z
        StyledLabel {
            Layout.column: 0
            Layout.row: 4
            text: "factor-z:"
        }
        StyledTextField {
            Layout.column: 1
            Layout.row: 4
            id: factorZ
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "1.0"
        }
    }
}
