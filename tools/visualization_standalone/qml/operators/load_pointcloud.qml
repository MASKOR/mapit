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

    function fromParameters(params) {
        parameters = params

    }

    //// out ////
    property bool valid: false
    property var parameters: {}

    //// UI ////
    ColumnLayout {
        anchors.fill: parent
        height: root.height
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
            Layout.fillHeight: true
            Text {
                Layout.alignment: Qt.AlignTop
                text: "Target:"
                color: palette.text
                renderType: Text.NativeRendering
            }
            EntityChooser {
                id: entityChooser
                Layout.fillWidth: true
                Layout.fillHeight: true
                currentCheckout: root.currentCheckout
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
        SystemPalette {
            id: palette
        }
    }
}
