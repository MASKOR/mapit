import QtQuick 2.4
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtQuick.Window 2.2 as Wnd
import QtGraphicalEffects 1.0

import pcl 1.0

import fhac.upns 1.0 as UPNS

import QtQuick 2.0 as QQ2

import "panes"

MainMenubar {
    id: window
    objectName: "mainWindow"
    title: qsTr("Mapit Visualization")
    width: appStyle.windowWidth
    height: appStyle.windowHeight
    onWidthChanged: {
        appStyle.windowWidth = width
    }
    onHeightChanged: {
        appStyle.windowHeight = height
    }

    visible: true
    UPNS.Repository {
        id: globalRepository
        url: "./demo_repository" // InitDemo
    }

    AppStyle {
        id: appStyle
        visible: false
    }
    ColumnLayout {
        anchors.fill: parent
        StyledSplitView {
            orientation: Qt.Horizontal
            Layout.fillWidth: true
            Layout.fillHeight: true
            LeftPanels {
                id: leftPanels
                Layout.fillHeight: true
                Layout.minimumWidth: 50
                width: appStyle.splitViewLeftWidth
                onWidthChanged: {
                    appStyle.splitViewLeftWidth = width
                }
            }
            SceneView {
                id: sceneView
                Layout.minimumWidth: 150
                Layout.fillWidth: true
                Layout.fillHeight: true
                currentEntitydata: leftPanels.currentEntitydata
                currentEntitydataTransform: leftPanels.currentEntitydataTransform
                visibleEntityItems: leftPanels.visibleElems
            }
            BottomPanels {
                Layout.fillHeight: true
                Layout.minimumWidth: 50
                width: appStyle.splitViewRightWidth
                currentOperator: leftPanels.currentOperator
                currentCheckout: leftPanels.currentCheckout
                currentEntityPath: leftPanels.currentEntityPath
                onWidthChanged: {
                    appStyle.splitViewRightWidth = width
                }
            }
        }
    }

    SystemPalette {
        id: palette
    }
}
