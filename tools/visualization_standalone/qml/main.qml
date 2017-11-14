import QtQuick 2.4
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtQuick.Window 2.2 as Wnd
import QtGraphicalEffects 1.0

import pcl 1.0

import fhac.upns 1.0 as UPNS

import "panes"

//Note: MenuBar and ApplicationWindow cannot be
//      seperated, "MainWindow" contains basically
//      the menubar, rest is done here.
MainWindow {
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
        url: appStyle.repositoryUrl
    }

    Item {
        id: applicationStateItem
        function selectOperator(name, props) {
            appStyle.tmpPrimitiveType = props.type
            leftPanels.selectOperator( name )
        }

        function selectEntity(path) {

        }
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
            Layout.margins: 0
            LeftPanels {
                id: leftPanels
                Layout.fillHeight: true
                Layout.minimumWidth: 50
                Layout.margins: 0
                currentFrameId: sceneView.currentFrameId
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
                applicationState: applicationStateItem
            }
            DetailPanels {
                Layout.fillHeight: true
                Layout.minimumWidth: 50
                Layout.margins: 0
                width: appStyle.splitViewRightWidth
                currentOperator: leftPanels.currentOperator
                currentPipeline: leftPanels.currentPipeline
                currentCheckout: leftPanels.currentCheckout
                currentEntityPath: leftPanels.currentEntityPath
                currentFrameId: sceneView.currentFrameId
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
