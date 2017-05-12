import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

import ".."

Item {
    id: root
    height: implicitHeight
    //// in ////
    property bool editable
    property var currentCheckout
    property string currentEntityPath

    function fromParameters(params) {
        fileChooser.filename = params.filename
        entityChooser.currentEntityPath = params.target
    }

    //// out ////
    property bool valid: fileChooser.valid && entityChooser.valid
    property var parameters: {
        "filename":fileChooser.filename,
        "target":entityChooser.currentEntityPath
    }

    //// UI ////
    ColumnLayout {
        anchors.fill: parent
        HelperOpenFile {
            id: fileChooser
            fileExtension: "pcd"
        }
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
