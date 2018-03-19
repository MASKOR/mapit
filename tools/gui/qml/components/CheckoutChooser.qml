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

import QtQuick 2.4
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtQuick.Window 2.2 as Wnd

import "../components"

StyledButton {
    id: root
    text: qsTr("Workspace")
    tooltip: qsTr("Open Dialog to choose a workspace to work on")
    property string currentWorkspaceName: appStyle.workspaceName
    onClicked: chooseCheckoutDialog.visible = !chooseCheckoutDialog.visible
    Wnd.Window {
        id: chooseCheckoutDialog
        width: 420
        height: 260
        minimumHeight: height
        maximumHeight: height
        minimumWidth: width
        maximumWidth: width
        flags: Qt.Dialog
        title: qsTr("Choose Workspace")
        color: appStyle.backgroundColor
        ColumnLayout {
            anchors.fill: parent

            ListView {
                id: checkoutList
                delegate: RowLayout {
                        Image {
                            source: "image://icon/asset-green"
                        }
                        StyledLabel {
                            text: globalRepository.workspaceNames[index]
                            MouseArea {
                                anchors.fill: parent
                                onClicked: checkoutList.currentIndex = index
                            }
                        }
                    }

                model: globalRepository.workspaceNames
                highlight: Rectangle { color: appStyle.selectionColor }

                Layout.fillWidth: true
                Layout.fillHeight: true
            }
            QCtl.Button {
                text: "+"
                onClicked: {
                    newCheckoutDialog.visible = !newCheckoutDialog.visible
                }
                Wnd.Window {
                    id: newCheckoutDialog
                    width: 420
                    height: 260
                    minimumHeight: height
                    maximumHeight: height
                    minimumWidth: width
                    maximumWidth: width
                    flags: Qt.Dialog
                    title: "Create new workspace"
                    color: appStyle.backgroundColor
                    GridLayout {
                        StyledLabel {
                            text: "Branchname"
                            Layout.column: 0
                            Layout.row: 0
                        }
                        StyledTextField {
                            id: branchnameTextedit
                            text: "master"
                            Layout.column: 1
                            Layout.row: 0
                        }
                        StyledLabel {
                            text: "Workspace name"
                            Layout.column: 0
                            Layout.row: 1
                        }
                        StyledTextField {
                            id: workspaceNameTextedit
                            Layout.column: 1
                            Layout.row: 1
                        }
                        StyledButton {
                            text: "Cancel"
                            onClicked: newCheckoutDialog.visible = false
                            Layout.column: 0
                            Layout.row: 2
                        }
                        StyledButton {
                            text: "Ok"
                            enabled: branchnameTextedit.text.trim().length !== 0
                                     && workspaceNameTextedit.text.trim().length !== 0
                            onClicked: {
                                globalRepository.createCheckout(branchnameTextedit.text, workspaceNameTextedit.text)
                                appStyle.workspaceName = workspaceNameTextedit.text
                                newCheckoutDialog.visible = false
                            }
                            Layout.column: 1
                            Layout.row: 2
                        }
                    }
                }
            }
            RowLayout {
                Layout.fillWidth: true
                StyledButton {
                    text: "Cancel"
                    onClicked: chooseCheckoutDialog.visible = false
                }
                StyledButton {
                    text: "Ok"
                    onClicked: {
                        root.currentWorkspaceName = globalRepository.workspaceNames[checkoutList.currentIndex];
                        appStyle.workspaceName = root.currentworkspaceName
                        chooseCheckoutDialog.visible = false;
                    }
                }
            }
        }
    }
}
