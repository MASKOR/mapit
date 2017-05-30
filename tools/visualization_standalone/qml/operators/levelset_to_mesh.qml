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
    property string currentEntityPath

    function fromParameters(params) {
        entityChooser.currentEntityPath = params.input
    }

    //// out ////
    property bool valid: entityChooser.valid
    property var parameters: {
        "input" : entityChooser.currentEntityPath,
        "output" : entityChooser.currentEntityPath + "-ply"
    }

    //// UI ////
    ColumnLayout {
        anchors.fill: parent
        height: root.height
        HelperTarget {
            id: entityChooser
            currentEntityPath: root.currentEntityPath
            dialogRoot: root
        }
        Item {
            Layout.fillHeight: true
        }
    }
}
