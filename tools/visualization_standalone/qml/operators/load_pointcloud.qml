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
        parameters = params

    }

    //// out ////
    property bool valid: fileNamePcd.text != "" && entityChooser.currentEntityPath != ""
    property var parameters: {
        "filename":fileNamePcd.text,
        "target":entityChooser.currentEntityPath
    }

    //// UI ////
    ColumnLayout {
        anchors.fill: parent
        RowLayout {
            Layout.fillWidth: true
            TextField {
                id: fileNamePcd
            }
            Button {
                text: "Open"
                onClicked: {
                    openPcdFileDialog.open()
                }
            }
        }
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
                currentEntityPath: root.currentEntityPath
            }
        }
        FileDialog {
            id: openPcdFileDialog
            title: "Open Pcd"
            selectExisting: true
            selectFolder: false
            selectMultiple: false
            onAccepted: {
                var filename = fileUrl.toString()
                filename = filename.replace(/^(file:\/{2})/,"")
                fileNamePcd.text = filename
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
