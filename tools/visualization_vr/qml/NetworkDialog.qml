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
import QtQuick.Controls 1.4

import "qrc:/qml/network"

ApplicationWindow {
    visible: true
    id: connectRealtimeMultiviewDialog
    objectName: "connectRealtimeMultiviewDialog"
    //flags: Qt.Tool
    title: qsTr( "Choose Mapit Realtime Server" )
    width: 420
    height: 400
    minimumWidth: width
    maximumWidth: width
    property alias url: clientUrl.text
    property alias peername: peernameTextfield.text
    property var mapitClient: globalMapitClient

    GridLayout {
        id: connectRealtimeMultiviewGrid
        anchors.fill: parent

        Label {
            text: qsTr( "Url" )
            Layout.column: 0
            Layout.row: 0
        }
        TextField {
            id: clientUrl
            //text: "ws://149.201.37.81:55511"
            text: "ws://127.0.0.1:55511"
            Layout.column: 1
            Layout.row: 0
            Layout.fillWidth: true
            //onTextChanged: mapitClient.url = text
        }
        Button {
            text: "Cancel"
            onClicked: {
                //connectRealtimeMultiviewDialog.visible = false
            }
            Layout.column: 0
            Layout.row: 2
        }
        Button {
            text: qsTr( "Ok/Connect" )
            enabled: clientUrl.text.trim().length !== 0
                     && clientUrl.text.trim().length !== 0
            onClicked: {
                activator.activat = true
                //connectRealtimeMultiviewDialog.mapitClient.active = true
            }
            Layout.column: 1
            Layout.row: 2
        }
        RowLayout {
            Label {
                text: qsTr( "Name" )
            }
            TextField {
                id: peernameTextfield
                text: "VR Bob" + Date.now()
                onTextChanged: mapitClient.ownState.peername = text
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
            Text {
                text: "Realtime objects:"
            }
            Repeater {
                model: connectRealtimeMultiviewDialog.mapitClient.state.realtimeObjects.count
                RowLayout {
                    Label {
                        property var obj: connectRealtimeMultiviewDialog.mapitClient.state.realtimeObjects.get(index)
                        property var owner: obj ? connectRealtimeMultiviewDialog.mapitClient.state.peerToPeerState[obj.peerOwner] : null
                        text: obj ? obj.ident + ", <b>Owner</b>: " + (owner ? owner.peername : "<loading>") + ", <b>Type</b>:  " + obj.type + (owner && owner.isHost ? "(<b>Host</b>)" : "") : "loading"
                        //tooltip: "" + obj.peerOwner + "<br>" + obj.tf
                        renderType: Text.NativeRendering
                    }
                }
            }
            Text {
                text: "Visible Network Entities:"
            }
            Repeater {
                model: connectRealtimeMultiviewDialog.mapitClient.state.visibleEntityInfosList.count
                RowLayout {
                    Label {
                        property var obj: connectRealtimeMultiviewDialog.mapitClient.state.visibleEntityInfosList.get(index)
                        text: obj.path
                        //tooltip: "" + obj.peerOwner + "<br>" + obj.tf
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
