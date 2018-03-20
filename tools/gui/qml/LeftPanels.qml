/*******************************************************************************
 *
 * Copyright 2017-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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
import QtQuick 2.0 as QQ2
import fhac.mapit 1.0 as Mapit
import "panes"
import "components"
import "network" // only for EntityVisualInfo in contextMenu

Item {
    id: root
    property alias treeView: treeViewWorkspace
//    property alias currentOperator: operatorListPane.currentOperator
//    property alias currentPipeline: operatorListPane.currentPipeline
//    property alias visibleElems: treeViewWorkspace.visibleElems
//    property string currentEntityPath: treeViewWorkspace.currentIndex && treeViewWorkspace.model.data(treeViewWorkspace.currentIndex, Mapit.RootTreeModel.NodeTypeRole) === Mapit.RootTreeModel.EntityNode ? treeViewWorkspace.model.data(treeViewWorkspace.currentIndex, Qt.ToolTipRole) : ""
//    property string currentFrameId

//    onCurrentEntityPathChanged: appStyle.tmpCurrentEditEntity = currentEntityPath

    ColumnLayout {
        anchors.fill: parent
        id: controlColumn

        StyledSplitView {
            orientation: Qt.Vertical
            Layout.fillWidth: true
            Layout.fillHeight: true
            ColumnLayout {
                Layout.fillWidth: true
                spacing: appStyle.controlMargin
                StyledHeader {
                    Layout.fillWidth: true
                    id: headerOpPane
                    text: qsTr("Operators")
                    iconSource: "image://material/ic_functions"
                }
                OperatorListPane {
                    Layout.fillWidth: true
                    id: operatorListPane
                    Layout.fillHeight: true
                    Layout.minimumHeight: 160
                    Layout.preferredHeight: 500
                    Layout.leftMargin: appStyle.controlMargin
                    Layout.rightMargin: appStyle.controlMargin
                    visible: headerOpPane.checked
                    pipelines: ListModel {
                        ListElement { displayName: "place_primitive" }
                    }
                    Connections {
                        target: globalApplicationState
                        onCurrentDetailDialogChanged: {
                            operatorListPane.selectItemByName(globalApplicationState.currentDetailDialog)
                        }
                    }
                    onCurrentDetailDialogPathChanged: globalApplicationState.currentDetailDialog = currentDetailDialogPath
                    onCurrentExecutableIsPipelineChanged: globalApplicationState.currentDetailDialogHasExecuteButton = currentExecutableIsPipeline
                }
                QCtl.Action {
                    id: actionShowHide
                    property var myVisualInfo: treeViewWorkspace.getVisualInfoForPath(contextMenu.path, contextMenu.itemIsEntity)
                    text: "&Toggle visibility"
                    enabled: myVisualInfo !== null && myVisualInfo !== undefined
                    shortcut: "T"
                    onTriggered: {
                        myVisualInfo.isVisible = !myVisualInfo.isVisible
                    }
                }
                QCtl.Action {
                    id: actionRename
                    text: "&Rename"
                    enabled: false
                    shortcut: "R"
                    onTriggered: {
                    }
                }
                QCtl.Action {
                    id: actionDelete
                    text: "&Delete"
                    enabled: false
                    shortcut: "D"
                    onTriggered: {
                    }
                }
            }
            ColumnLayout {
                Layout.preferredHeight: 300
                Layout.fillWidth: true
                spacing: appStyle.controlMargin
                StyledHeader {
                    Layout.fillWidth: true
                    id: headerWorkspace
                    text: qsTr("Workspace")
                    iconSource: "image://material/ic_layers"
                }
                ColumnLayout {
                    Layout.fillWidth: true
                    Layout.leftMargin: appStyle.controlMargin
                    Layout.rightMargin: appStyle.controlMargin
                    spacing: appStyle.controlMargin
                    visible: headerWorkspace.checked
                    RowLayout {
                        Layout.fillWidth: true
                        StyledLabel {
                            text: globalApplicationState.currentWorkspace.name
                            Layout.fillWidth: true
                        }
                        WorkspaceChooser {
                            width: implicitWidth
                            onCurrentWorkspaceNameChanged: globalApplicationState.currentWorkspaceName = currentWorkspaceName
                        }
                    }
                    StyledLabel {
                        visible: !globalRepository.isLoaded
                        text: "No Repository loaded"
                    }
                    WorkspaceTreeView {
                        visible: globalRepository.isLoaded
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.bottomMargin: appStyle.controlMargin
                        id: treeViewWorkspace
                        onCurrentEntityPathChanged: globalApplicationState.currentEntityPath = currentEntityPath
                        contextMenu: QCtl.Menu {
                            id: contextMenu
                            // These properties are set when the popup opens
                            property var index
                            property string path
                            property bool itemIsEntity
                            QCtl.MenuItem {
                                visible: contextMenu.itemIsEntity
                                action: actionShowHide
                            }
                            QCtl.MenuItem {
                                action: actionRename
                            }
                            QCtl.MenuItem {
                                action: actionDelete
                            }
                        }
                    }
                }
            }
        }
    }
}
