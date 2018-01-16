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
        entityChooserSource.currentEntityPath = params.target
    }

    //// out ////
    property bool valid:   true
    property var parameters: {
        "source":    entityChooserSource.currentEntityPath,
        "target":    entityChooserTarget.currentEntityPath
    }

    //// UI ////
    GridLayout {
        z: 100
        // source
        StyledLabel {
            Layout.column: 0
            Layout.row: 1
            text: "Source:"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 1
            z: 100
            id: entityChooserSource
            Layout.fillWidth: true
            currentEntityPath: root.currentEntityPath
        }
        // target
        StyledLabel {
            Layout.column: 0
            Layout.row: 2
            text: "Target:"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 2
            z: 99
            id: entityChooserTarget
            Layout.fillWidth: true
        }
    }
}
