import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

import ".."

Item {
    id: root
    //// in ////
    property bool editable
    property var currentCheckout
    property string currentEntity

    function fromParameters(params) {
        entityChooser.currentEntityPath = params.target
    }

    //// out ////
    property bool valid: entityChooser.currentEntityPath != ""
    property var parameters: {
        "target":entityChooser.currentEntityPath
    }

    //// UI ////
    ColumnLayout {
        anchors.fill: parent
        RowLayout {
            Layout.fillWidth: true
            Text {
                Layout.alignment: Qt.AlignTop
                text: "Target:"
                color: palette.text
                renderType: Text.NativeRendering
            }
            EntityChooser {
                id: entityChooser
                Layout.fillWidth: true
                currentCheckout: root.currentCheckout
                currentEntityPath: root.currentEntity
            }
        }
        Item {
            Layout.fillHeight: true
        }
        SystemPalette {
            id: palette
        }
    }
}
