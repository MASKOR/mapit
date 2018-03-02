/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

import QtQuick 2.5
import QtQuick.Layouts 1.1
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Dialogs 1.2
import FileIO 1.0

import "../components"

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
