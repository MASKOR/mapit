import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

import ".."
import "../components"

RowLayout {
    Layout.fillWidth: true
    property string fileExtension
    property alias filename: fileName.text
    property bool valid: filename != ""
    StyledTextField {
        id: fileName
    }
    StyledButton {
        text: "Open"
        onClicked: {
            openFileDialog.open()
        }
    }
    FileDialog {
        id: openFileDialog
        title: "Open " + fileExtension
        selectExisting: true
        selectFolder: false
        selectMultiple: false
        onAccepted: {
            var filename = fileUrl.toString()
            filename = filename.replace(/^(file:\/{2})/,"")
            fileName.text = filename
        }
    }
}
