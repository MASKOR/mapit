/*******************************************************************************
 *
 * Copyright      2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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

import QtQuick 2.9
import QtQuick.Window 2.3
import QtQuick.Layouts 1.3

import "../components"

Window {
    id: connectRealtimeMultiviewDialog
    flags: Qt.Tool
    title: qsTr( "Realtime Network" )
    color: appStyle.backgroundColor
    width: 420
    height: 400
    minimumWidth: width
    maximumWidth: width
    property alias isServer: isServerCheckbox.checked
    property int port: parseInt(portTextfield.text)
    property alias url: clientUrl.text
    property alias peername: peernameTextfield.text
    GridLayout {
        id: connectRealtimeMultiviewGrid
        anchors.fill: parent

        StyledLabel {
            text: qsTr( "Url" )
            Layout.column: 0
            Layout.row: 0
        }
        StyledTextField {
            id: clientUrl
            text: webServer.url
            enabled: !isServerCheckbox.checked
            onEnabledChanged: text = webServer.url
            Layout.column: 1
            Layout.row: 0
            Layout.fillWidth: true
        }
        StyledCheckBox {
            id: isServerCheckbox
            text: qsTr( "Server" )
            Layout.column: 0
            Layout.row: 1
        }
        RowLayout {
            StyledLabel {
                text: qsTr( "Port" )
            }
            StyledTextField {
                id: portTextfield
                validator: IntValidator {
                    bottom: 0
                    top: 65535
                }
                text: "55511"
            }
            Layout.column: 1
            Layout.row: 1
            Layout.fillWidth: true
        }
        StyledButton {
            text: "Cancel"
            onClicked: {
                webServer.listen = false;
                connectRealtimeMultiviewDialog.visible = false
            }
            Layout.column: 0
            Layout.row: 2
        }
        StyledButton {
            text: qsTr( "Ok" )
            enabled: clientUrl.text.trim().length !== 0
                     && clientUrl.text.trim().length !== 0
            onClicked: {
                if(isServerCheckbox.checked) {
                    //globalRepository.url = "." //TODO: temp, DBG
                    webServer.listen = true
                    webServer.accept = true
                    mapitClient.active = true
                } else {
                    mapitClient.active = true
                }

                //connectRealtimeMultiviewDialog.visible = false
            }
            Layout.column: 1
            Layout.row: 2
        }
        RowLayout {
            StyledLabel {
                text: qsTr( "Name" )
            }
            StyledTextField {
                id: peernameTextfield
                text: "Alice" + Date.now()
            }
            Layout.column: 0
            Layout.row: 3
            Layout.fillWidth: true
        }
        ColumnLayout {
            Layout.column: 0
            Layout.columnSpan: 2
            Layout.row: 4
            Layout.fillWidth: true
            Layout.fillHeight: true
            //Layout.minimumHeight: 200
            height: 200
            StyledLabel {
                text: "Realtime objects:"
            }
            Repeater {
                model: mapitClient.state.realtimeObjects.count
                RowLayout {
                    StyledLabel {
                        property RealtimeObject obj: mapitClient.state.realtimeObjects.get(index)
                        property var owner: mapitClient.state.peerToPeerState[obj.peerOwner]
                        text: obj.ident + ", <b>Owner</b>: " + owner.peername + ", <b>Type</b>:  " + obj.type + (owner.isHost ? "(<b>Host</b>)" : "")
                        tooltip: "" + obj.peerOwner + "<br>" + obj.tf
                        renderType: Text.NativeRendering
                    }
                }
            }
            StyledLabel {
                text: "Visible Network Entities:"
            }
            Repeater {
                model: mapitClient.state.visibleEntityInfosList.count
                RowLayout {
                    StyledLabel {
                        property var obj: mapitClient.state.visibleEntityInfosList.get(index)
                        text: obj.path
                        tooltip: "" + obj.peerOwner + "<br>" + obj.tf
                        renderType: Text.NativeRendering
                    }
                }
            }
            Item {
                Layout.fillHeight: true
            }
        }
    }
}
