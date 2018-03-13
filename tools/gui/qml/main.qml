/*******************************************************************************
 *
 * Copyright 2015-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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
import QtGraphicalEffects 1.0

import pcl 1.0

import fhac.upns 1.0 as UPNS

import "panes"
import "components"

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

    Item {
        // overlay for popups, tooltips
        z: 10000
        id: overlay
        anchors.fill: parent
//        MouseArea {
//            id: globalBlur
//            anchors.fill: parent
//            hoverEnabled: false

//            preventStealing: false
//            propagateComposedEvents: true
//            //z: -10000
//            onClicked: mouse.accepted = false
//            onPressed: {
//                focus = true
//                mouse.accepted = false
//            }
//        }
    }

    ApplicationState {
        // most components communicate over this item. E.g. currently selected Entity
        // Moreover there is AppStyle, which also holds parts of the application state
        id: globalApplicationState
        visible: false
        visibleEntityModel: leftPanels.treeView.visibleEntityModel
        allVisualInfoModel: leftPanels.treeView.allVisualInfoModel
    }

    AppStyle {
        // Components beginning with "Styled*" use AppStyle for coloring, sizing, ...
        // This is a practical solution for this application without overengineering
        // or coming up with new ways of styling. Do not use SystemPalette directly!
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
                visibleEntityModel: globalApplicationState.visibleEntityModel
                allVisualInfoModel: globalApplicationState.allVisualInfoModel
                onPointSizeChanged: sendDelayedTimer.start()
                onShaderVarChanged: sendDelayedTimer.start()
                onShaderVar2Changed: sendDelayedTimer.start()
                onRenderStyleChanged: sendDelayedTimer.start()
            }
            Timer {
                id: sendDelayedTimer
                interval: 10
                repeat: false
                onTriggered: sceneView.mapitClient.sendOwnState()
            }

            DetailPanels {
                Layout.fillHeight: true
                Layout.minimumWidth: 50
                Layout.margins: 0
                width: appStyle.splitViewRightWidth
                onWidthChanged: {
                    appStyle.splitViewRightWidth = width
                }
            }
        }
    }
}
