import QtQuick 2.5
import QtQuick.Layouts 1.1
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Dialogs 1.2
import FileIO 1.0

Dialog {
    title: qsTr( "About" )
    modality: Qt.WindowModal
    visible: false
    ColumnLayout {
        anchors.fill: parent

        StyledLabel {
            Layout.fillWidth: true
            textFormat: Text.RichText
            text: "About"
        }
        StyledLabel {
            Layout.fillWidth: true
            textFormat: Text.RichText
            text: "EditorCameraController is used from Qt3D Editor.\n" +
                  "<a href=\"https://github.com/qt-labs/qt3d-editor\">https://github.com/qt-labs/qt3d-editor</a>"
        }
        StyledButton {
            Layout.fillWidth: true
            text: "Show License"
            onClicked: qt3dEditorDialogLicense.open()
            Dialog {
                title: "LICENSE.GPL3-EXCEPT (qt3d-editor)"
                id: qt3dEditorDialogLicense
                standardButtons: StandardButton.Ok
                contentItem: Rectangle {
                    anchors.fill: parent
                    id: licRect
                    color: appStyle.backgroundColor
                    TextArea {
                        anchors.fill: parent
                        textColor: appStyle.textColor
                        font.family: appStyle.labelFontFamily
                        font.weight: appStyle.labelFontWeight
                        font.pixelSize: appStyle.labelFontPixelSize
                        style: TextAreaStyle {
                            textColor: appStyle.textColor
                            selectionColor: appStyle.selectionColor
                            selectedTextColor: appStyle.textColor
                            backgroundColor: appStyle.backgroundColor
                            renderType: Text.NativeRendering
                        }
                        text: qt3dEditorLicense.read()
                        FileIO {
                            id: qt3dEditorLicense
                            source: ":/license/LICENSE.GPL3-EXCEPT"
                            onError: console.log(msg)
                        }
                    }
                }
            }
        }
        StyledLabel {
            Layout.fillWidth: true
            textFormat: Text.RichText
            text: "3D Primitive Icon Set by Andrew Ray."
        }
    }
}
