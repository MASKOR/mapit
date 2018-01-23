//TODO: Rename this file
import QtQuick 2.4
import QtQml 2.2
import QtQuick.Controls 1.3
import QtQuick.Controls.Styles 1.4
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1

import "panes"
import "components"
import "network"
import fhac.upns 1.0 as UPNS

ApplicationWindow {
    id: mainWindow
    color: appStyle.backgroundColor
    onClosing: Qt.quit()
    Window {
        id: executeOperatorDialog
        flags: Qt.Tool | Qt.WindowCloseButtonHint
        title: qsTr( "Execute Operator" )
        color: appStyle.backgroundColor
        width: 420
        height: 260
        minimumHeight: height
        maximumHeight: height
        minimumWidth: width
        maximumWidth: width
        property alias detailDialogPath: opPane.detailDialogPath
        property alias currentCheckout: opPane.currentCheckout
        property alias currentEntityPath: opPane.currentEntityPath
        RowLayout {
            anchors.fill: parent
            OperatorPane {
                id: opPane
                Layout.fillWidth: true
                Layout.fillHeight: true
            }
        }
    }
    NetworkDialog {
        id: connectRealtimeMultiviewDialog
    }

    MapitClient {
        id: mapitClient
        url: connectRealtimeMultiviewDialog.url
        Component.onCompleted: globalApplicationState.mapitClient = mapitClient
        ownState: MapitMultiviewPeerState {
            id: multiviewPeerState
            peername: connectRealtimeMultiviewDialog.peername
            onPeernameChanged: mapitClient.sendOwnState()
            isHost: connectRealtimeMultiviewDialog.isServer
            repositoryPort: repoServer.port
            checkoutName: globalApplicationState.currentCheckoutName
            realtimeObjects: [
                RealtimeObject {
                    id: theRto
                    tf: sceneView.camera.viewMatrix
                    type: "frustum"
                    additionalData: { "aspect": sceneView.camera.aspectRatio, "fov": sceneView.camera.fieldOfView }
                }
            ]
        }
    }
    Connections {
        target: mapitClient.state
        enabled: !connectRealtimeMultiviewDialog.isServer
        onCheckoutNameChanged: {
            globalApplicationState.currentCheckoutName = mapitClient.state.checkoutName
        }
        onRepositoryUrlChanged: {
            globalRepository.url = mapitClient.state.repositoryUrl
        }
    }

    UPNS.RepositoryServer {
        id: repoServer
        running: connectRealtimeMultiviewDialog.isServer
        repository: globalRepository
        onRunningChanged: console.log("Repositoryserver started at port " + port)
    }
    Connections {
        target: sceneView.camera
        onViewMatrixChanged: mapitClient.sendOwnState()
        onAspectRatioChanged: mapitClient.sendOwnState()
        onFieldOfViewChanged: mapitClient.sendOwnState()
    }

    MapitServer {
        id: webServer
        port: connectRealtimeMultiviewDialog.port
        listen: true
        accept: true
    }

    Window {
        id: openRepoDialog
        flags: Qt.Tool
        title: qsTr( "Choose Checkout" )
        color: appStyle.backgroundColor
        width: 420
        height: openRepoGrid.implicitHeight
        minimumHeight: height
        maximumHeight: height
        minimumWidth: width
        maximumWidth: width
//        StyledLabel {
//            wrapMode: Text.WordWrap
//            text: "At the moment the repository is fixed an can be changed using command line options on start. type \"mapit_viewer help\" for more information"
//        }
        GridLayout {
            id: openRepoGrid
            anchors.fill: parent
            StyledLabel {
                text: qsTr( "Url or Filename" )
                Layout.column: 0
                Layout.row: 0
            }
            StyledTextField {
                id: repoUrl
                text: "tcp://"
                Layout.column: 1
                Layout.row: 0
                Layout.fillWidth: true
            }
            StyledButton {
                text: "Cancel"
                onClicked: openRepoDialog.visible = false
                Layout.column: 0
                Layout.row: 1
            }
            StyledButton {
                text: qsTr( "Ok" )
                enabled: repoUrl.text.trim().length !== 0
                         && repoUrl.text.trim().length !== 0
                onClicked: {
                    globalRepository.url = repoUrl.text
                    appStyle.repositoryUrl = repoUrl.text
                    openRepoDialog.visible = false
                }
                Layout.column: 1
                Layout.row: 1
            }
        }
    }
    Binding {
        target: appStyle
        property: "showGrid"
        value: showGrid.checked
    }
    //property alias fixUpvector: fixedUpvec.checked
    //property alias invertYAxis: invertY.checked
    //property alias showCenterCross: showCenter.checked
    property alias enableVr: vrModeEnabled.checked
    property alias mirrorEnabled: vrMirrorOff.vrMirrorEnabled
    property alias mirrorDistorsion: vrMirrorDistorsion.checked
    property alias mirrorRightEye: vrMirrorRight.checked
    property alias mirrorLeftEye: vrMirrorLeft.checked
    property bool uiEnabled
    property bool vrAvailable: true
    menuBar: MenuBar {
        id: menuBar

        style: MenuBarStyle {
            id: menuBarStyle
            background: Rectangle {
                height: appStyle.controlHeightContainer; color: appStyle.backgroundColor
            }
            itemDelegate:  Component {
                id: theItemDelegate
                MouseArea {
                    id: itemMa
                    implicitHeight: appStyle.controlHeightContainer
                    implicitWidth: menuItemLabel.implicitWidth+appStyle.controlMargin*2
                    hoverEnabled: true
                    Item {
                        anchors.fill: parent
                        StyledLabel {
                            id: menuItemLabel
                            anchors.centerIn: parent
                            text: menuBarStyle.formatMnemonic(styleData.text, itemMa.containsMouse)
                            verticalAlignment: Text.AlignVCenter
                            z: 100
                        }
                        Rectangle {
                            visible: itemMa.containsMouse
                            anchors.fill: parent
                            color: styleData.selected ? appStyle.backgroundHighlightColor : appStyle.backgroundColor
                        }
                    }
                }
            }
            menuStyle: MenuStyle {
                id: menuStyle
                itemDelegate.background: Rectangle {
                    color: styleData.selected ? appStyle.backgroundHighlightColor : appStyle.backgroundColor
                }
                itemDelegate.label: StyledLabel {
                    text: menuStyle.formatMnemonic(styleData.text, true)
                }
                itemDelegate.checkmarkIndicator: Rectangle {
                    implicitHeight: 10
                    implicitWidth: 10
                    color: styleData.checked
                           ? appStyle.backgroundHighlightColor
                           : appStyle.itemBackgroundColor
                    border.width: 1
                    border.color: appStyle.buttonBorderColor
                    smooth: false
                    radius: 0
                    Rectangle {
                        anchors.fill: parent
                        visible: styleData.checked
                        color: appStyle.selectionColor
                        border.color: appStyle.selectionBorderColor
                        radius: 0
                        anchors.margins: 4
                    }
                }
            }
        }

        Menu {
            title: qsTr("&File")
            MenuItem {
                text: qsTr("&Open Repository")
                onTriggered: openRepoDialog.show()
            }
            MenuItem {
                text: qsTr("Realtime Multiview &Connection")
                onTriggered: connectRealtimeMultiviewDialog.show()
            }
            MenuItem {
                text: qsTr("&Commit")
                enabled: false
            }
            MenuItem {
                text: qsTr("E&xit")
                onTriggered: Qt.quit();
            }
        }
        Menu {
            title: qsTr("&Edit")
            MenuItem {
                text: qsTr("&Copy")
                onTriggered: console.log("not yet implemented.");
            }
            MenuItem {
                text: qsTr("&Paste")
                onTriggered: console.log("not yet implemented.");
            }
        }
        Menu {
            id: operatorsMenu
            title: qsTr("&Operators")
            Instantiator {
                id: operatorInstantiator
                model: globalRepository.operators
                onObjectAdded: operatorsMenu.insertItem( index, object )
                onObjectRemoved: operatorsMenu.removeItem( object )
                onModelChanged: console.log("Model Changed" + globalRepository.operators.length)
                MenuItem {
                    text: operatorInstantiator.model[index].moduleName
                    onTriggered: {
                        executeOperatorDialog.currentCheckout = globalApplicationState.currentCheckout
                        executeOperatorDialog.currentEntityPath = globalApplicationState.currentEntityPath
                        executeOperatorDialog.detailDialogPath = "../operators/" + operatorInstantiator.model[index]
                        executeOperatorDialog.show()
                    }
                }
            }
        }
        Menu {
            title: qsTr("&View")
    //            MenuItem {
    //                id: detailHiItem
    //                text: qsTr("&Hi Detail")
    //                checkable: true
    //                checked: true
    //                onCheckedChanged: {
    //                }
    //            }
    //            MenuItem {
    //                id: detailMidtem
    //                text: qsTr("&Mid Detail")
    //                checkable: true
    //                checked: false
    //                onCheckedChanged: {
    //                }
    //            }
    //            MenuItem {
    //                id: detailLoItem
    //                text: qsTr("&Low Detail")
    //                checkable: true
    //                checked: false
    //                onCheckedChanged: {
    //                }
    //            }
    //        MenuSeparator { }
    //            MenuItem {
    //                id: executeItem
    //                text: qsTr("Lower Detail when &moving")
    //                enabled: false
    //                onTriggered: {
    //                }
    //            }
    //        MenuSeparator { }
    //        MenuItem {
    //            id: showCenter
    //            text: qsTr("Show Center &Cross")
    //            checkable: true
    //            checked: true
    //        }
            MenuItem {
                id: showGrid
                text: qsTr("Show &Grid")
                checkable: true
                checked: appStyle.showGrid

    //            onToggled: {
    //                appStyle.showGrid = checked
    //                appStyle.emitThemeChanged()
    //            }
            }
            MenuSeparator { }
    //            MenuItem {
    //                id: centerFixed
    //                text: qsTr("Torso Yaw fixed to view")
    //                checkable: true
    //                checked: true
    //            }
    //        MenuItem {
    //            id: fixedUpvec
    //            text: qsTr("Fixed Upvector")
    //            checkable: true
    //            checked: true
    //        }
            MenuItem {
                id: yAxisIsUp
                text: qsTr("Y Axis is Up")
                checkable: true
                checked: appStyle.coordinateSystemYPointsUp
            }
            MenuSeparator { }
            MenuItem {
                id: vrModeEnabled
                text: qsTr("Enable VR")
                enabled: mainWindow.vrAvailable
                checkable: mainWindow.vrAvailable
                checked: true
            }
            Menu {
                id: vrMirror
                title: qsTr("Mirror VR")
                enabled: vrModeEnabled.checked
                ExclusiveGroup {
                    id: mirrorGroup
                }
                MenuItem {
                    property bool vrMirrorEnabled: !vrMirrorOff.checked
                    id: vrMirrorOff
                    text: qsTr("None")
                    checkable: mainWindow.uiEnabled
                    checked: false
                    exclusiveGroup: mirrorGroup
                }
                MenuItem {
                    id: vrMirrorDistorsion
                    text: qsTr("Distorsion")
                    checkable: mainWindow.uiEnabled
                    checked: true
                    exclusiveGroup: mirrorGroup
                }
                MenuItem {
                    id: vrMirrorRight
                    text: qsTr("Right Eye")
                   // enabled: false
                    checkable: mainWindow.uiEnabled // not yet available
                    checked: false
                    exclusiveGroup: mirrorGroup
                }
                MenuItem {
                    id: vrMirrorLeft
                    text: qsTr("Left Eye")
                    //enabled: false
                    checkable: mainWindow.uiEnabled // not yet available
                    checked: false
                    exclusiveGroup: mirrorGroup
                }
            }
        }
        Menu {
            title: qsTr("&Window")
            MenuItem {
                text: qsTr("Show &Operations Pane")
                checkable: true
                checked: true
                onCheckedChanged: {
                }
            }
            MenuItem {
                text: qsTr("Show &Maps/Layers Pane")
                checkable: true
                checked: false
                onCheckedChanged: {
                }
            }
        }
    }
}
