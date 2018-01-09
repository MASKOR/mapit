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
        "target": ""
    }

    function beforeOperation() {
        var target = []
        for (var i = 0; i < inputRepeater.model.count; ++i) {
            if (inputRepeater.model.get(i).name) {
                target.push( inputRepeater.model.get(i).name )
            }
        }
        if (entityChooserInput.currentEntityPath) {
            target.push( entityChooserInput.currentEntityPath )
        }

        parameters["target"] = target;
    }

    //// UI ////
    GridLayout {
        z: 150
        // target
        Repeater {
            id: inputLableRepeater
            model: 0
            StyledLabel {
                Layout.column: 0
                Layout.row: 2 + (2 * index)
                text: "Target[" + index + "]"
            }
        }
        Repeater {
            id: inputRepeater
            model: ListModel {}
            EntityChooser {
                Layout.column: 1
                Layout.row: 2 + (2 * index)
                z: 200 - index
                id: entityChooserInputRepeated
                Layout.fillWidth: true
                currentEntityPath: inputRepeater.model.get(index).name
                onCurrentEntityPathChanged: {
                    inputRepeater.model.get(index).name = currentEntityPath
                }
            }
        }

        StyledLabel {
            Layout.column: 0
            Layout.row: 3 + (2 * inputRepeater.count)
            text: "Target[" + inputRepeater.count + "]"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 3 + (2 * inputRepeater.count)
            z: 198 - inputRepeater.count
            id: entityChooserInput
            Layout.fillWidth: true
            currentEntityPath: root.currentEntityPath
        }
        Connections {
            target: entityChooserInput.internalTextField
            onEditingFinished: {
                if (entityChooserInput.currentEntityPath) {
                    var entityLastName = entityChooserInput.currentEntityPath
                    entityChooserInput.currentEntityPath = ""

                    inputRepeater.model.append( { "name": entityLastName } )
                    inputLableRepeater.model = inputRepeater.count
                }
            }
        }
    }
}
