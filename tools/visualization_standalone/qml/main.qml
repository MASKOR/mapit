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

    ColumnLayout {
        anchors.fill: parent
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            LeftPanels {
                id: leftPanels
                Layout.fillHeight: true
                width: 200
            }
            SceneView {
                Layout.minimumWidth: 50
                Layout.fillWidth: true
                Layout.fillHeight: true
                currentEntitydata: leftPanels.currentEntitydata
                currentEntitydataTransform: leftPanels.currentEntitydataTransform
                visibleEntityItems: leftPanels.visibleElems
            }
        }
        BottomPanels {
            height: 200
            Layout.fillWidth: true
            currentOperator: leftPanels.currentOperator
            currentCheckout: leftPanels.currentCheckout
            currentEntity: "map/layer/entity"
        }
    }

    SystemPalette {
        id: palette
    }
}
