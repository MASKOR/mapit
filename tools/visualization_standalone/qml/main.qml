import QtQuick 2.4
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtQuick.Window 2.2 as Wnd
import QtGraphicalEffects 1.0

import pcl 1.0

import fhac.upns 1.0 as UPNS

import QtQuick 2.0 as QQ2

import "panes"

QCtl.ApplicationWindow {
    id: window
    title: qsTr("Map Visualization")
    width: 1200
    height: 800
    visible: true
    menuBar: MainMenubar {
        id: menubar
        //uiEnabled: drawingArea.renderdata.running
    }
//    UPNS.Repository {
//        id: repo
//        conf: "./repo.yaml"
//    }

    AppStyle {
        id:appStyle
        visible:false
    }
    ColumnLayout {
        anchors.fill: parent
        QCtl.SplitView {
            orientation: Qt.Horizontal
            Layout.fillWidth: true
            Layout.fillHeight: true
            LeftPanels {
                id: leftPanels
                Layout.fillHeight: true
                Layout.minimumWidth: 50
                width: 210
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
                width: 220
                currentOperator: leftPanels.currentOperator
                currentCheckout: leftPanels.currentCheckout
                currentEntityPath: leftPanels.currentEntityPath
            }
        }
    }

    SystemPalette {
        id: palette
    }
}
