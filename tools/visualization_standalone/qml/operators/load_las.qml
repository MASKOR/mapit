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
        parameters = params
        fileNamePcd.text = params.filename;
        ntityChooser.currentEntityPath = params.target;
        demeanCheckbox.checked = params.demean;
        normalizeCheckbox.checked = params.normalize;
    }

    //// out ////
    property bool valid: fileNamePcd.text != "" && entityChooser.currentEntityPath != ""
    property var parameters: {
        "filename":fileNamePcd.text,
        "target":entityChooser.currentEntityPath,
        //"demean": demeanCheckbox.checked,
        //"normalize": normalizeCheckbox.checked
    }

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
            z: 100
            StyledLabel {
                text: "Target:"
            }
            EntityChooser {
                id: entityChooser
                Layout.fillWidth: true
                currentCheckout: root.currentCheckout
                currentEntityPath: root.currentEntityPath
            }
        }
//        RowLayout {
//            Layout.fillWidth: true
//            StyledLabel {
//                text: "Demean:"
//            }
//            CheckBox {
//                id: demeanCheckbox
//                Layout.fillWidth: true
//            }
//        }
//        RowLayout {
//            Layout.fillWidth: true
//            StyledLabel {
//                text: "Normalize"
//            }
//            CheckBox {
//                id: normalizeCheckbox
//                Layout.fillWidth: true
//            }
//        }
        Item {
            Layout.fillHeight: true
        }
        FileDialog {
            id: openPcdFileDialog
            title: "Open Las"
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
