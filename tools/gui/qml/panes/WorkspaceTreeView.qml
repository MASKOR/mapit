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

import QtQuick 2.7
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtGraphicalEffects 1.0
import QtQml.Models 2.3

import fhac.mapit 1.0 as Mapit

import "../components"
import "qrc:/qml/network"

QCtl.TreeView {
    id: treeViewWorkspace
    property var currentWorkspace: globalApplicationState.currentWorkspace
    property var contextMenu
    // all objects, also cached and invisible
    property var allVisualInfoModel: ([])//ObjectModel {}
    // only visible entities
    property ListModel visibleEntityModel: ListModel {}
    property string currentEntityPath: selectionModel.currentIndex && treeViewWorkspace.model.data(selectionModel.currentIndex, Mapit.RootTreeModel.NodeTypeRole) === Mapit.RootTreeModel.EntityNode ? treeViewWorkspace.model.data(selectionModel.currentIndex, Qt.ToolTipRole) : ""
    signal visibleElemsUpdated
    alternatingRowColors: true
    headerVisible: false
    backgroundVisible: false
    //selectionMode: SelectionMode.SingleSelection
    selection: ItemSelectionModel {
        id: selectionModel
        model: treeViewWorkspace.model
    }
    function getVisualInfoForPath(path, isEntity) {
        if(!path) return
        for(var itmr=0 ; listSynchronizer.itemsToAddToAllVisualInfoModel.length > itmr ; ++itmr) {
            var current = listSynchronizer.itemsToAddToAllVisualInfoModel[itmr]
            if(current.path === path) {
                current.isEntity = isEntity
                return current
            }
        }
        for(var i=0 ; treeViewWorkspace.allVisualInfoModel.length > i ; ++i) {
            var current = treeViewWorkspace.allVisualInfoModel[i]
            if(current.path === path) {
                current.isEntity = isEntity
                return current
            }
        }
        var newObject = visualInfoComponent.createObject(treeViewWorkspace)
        newObject.path = path
        newObject.isEntity = isEntity
        newObject.onIsVisibleChanged.connect(function(isVisible) {
            if(!newObject.isEntity) return
            if(newObject.isVisible) {
                var found = false
                for(var i=0 ; i < treeViewWorkspace.visibleEntityModel.count ; ++i) {
                    if(treeViewWorkspace.visibleEntityModel.get(i).path === newObject.path) {
                        found = true
                        break
                    }
                }
                if(!found) {
                    for(var itmr2=0 ; itmr2 < listSynchronizer.itemsToAdd.length ; ++itmr2) {
                        if(listSynchronizer.itemsToAdd[itmr2].path === newObject.path) {
                            found = true
                            break
                        }
                    }
                }

                if(!found) {
                    var idx = -1
                    for(var i=0 ; i < treeViewWorkspace.allVisualInfoModel.length ; ++i) {
                        if(treeViewWorkspace.allVisualInfoModel[i].path === newObject.path) {
                            idx = i
                            break
                        }
                    }
                    listSynchronizer.itemsToAdd.push({path:newObject.path, idxInVisualInfoModel:idx})
                    listSynchronizer.start()
//                    treeViewWorkspace.visibleEntityModel.append({path:newObject.path, idxInVisualInfoModel:idx})
//                    var arr = treeViewWorkspace.visibleEntityModel
//                    arr.push({path:newObject.path, idxInVisualInfoModel:idx})
//                    treeViewWorkspace.visibleEntityModel = arr
                }
            } else {
                listSynchronizer.itemsToRemove.push(newObject)
                listSynchronizer.start()
//                console.log("DBG: getVisualInfoForPath: removing count: " + treeViewWorkspace.visibleEntityModel.count)
//                for(var i=0 ; i < treeViewWorkspace.visibleEntityModel.count ; ++i) {
//                    console.log("DBG: getVisualInfoForPath: treeViewWorkspace.visibleEntityModel index: " + i)
//                    if(treeViewWorkspace.visibleEntityModel.get(i).path === newObject.path) {
//                        console.log("DBG: getVisualInfoForPath: removed newObject.path: " + newObject.path)
//                        treeViewWorkspace.visibleEntityModel.remove(i)
//                        console.log("DBG: getVisualInfoForPath: deleted newObject.path: " + newObject.path)
////                        var arr = treeViewWorkspace.visibleEntityModel
////                        arr.splice(i, 1)
////                        treeViewWorkspace.visibleEntityModel = arr
//                    }
//                }
            }
        })
        listSynchronizer.itemsToAddToAllVisualInfoModel.push(newObject)
        listSynchronizer.start()
//        treeViewWorkspace.allVisualInfoModel.push(newObject)
        return newObject
    }
    Timer {
        // it is not safe to call treeViewWorkspace.visibleEntityModel.remove(i) while view are processed.
        id: listSynchronizer
        property var itemsToRemove: ([])
        property var itemsToAdd: ([])
        property var itemsToAddToAllVisualInfoModel: ([])
        // does not work because no bindings for array
        //running: itemsToAdd.length !== 0 || itemsToRemove.length !== 0
        interval: 100
        onTriggered: {
            var toRemoveIndex = listSynchronizer.itemsToRemove.length
            while(toRemoveIndex--) {
                var removeItem = listSynchronizer.itemsToRemove[toRemoveIndex]
                for(var i=0 ; i < treeViewWorkspace.visibleEntityModel.count ; ++i) {
                    if(treeViewWorkspace.visibleEntityModel.get(i).path === removeItem.path) {
                        treeViewWorkspace.visibleEntityModel.remove(i)
                    }
                }
                listSynchronizer.itemsToRemove.splice(toRemoveIndex, 1)
            }
            var toAddIndex = listSynchronizer.itemsToAdd.length
            while(toAddIndex--) {
                var addItem = listSynchronizer.itemsToAdd[toAddIndex]
                treeViewWorkspace.visibleEntityModel.append(addItem)
                listSynchronizer.itemsToAdd.splice(toAddIndex, 1)
            }
            var toAddIndex2 = listSynchronizer.itemsToAddToAllVisualInfoModel.length
            while(toAddIndex2--) {
                var addItem2 = listSynchronizer.itemsToAddToAllVisualInfoModel[toAddIndex2]
                treeViewWorkspace.allVisualInfoModel.push(addItem2)
                listSynchronizer.itemsToAddToAllVisualInfoModel.splice(toAddIndex2, 1)
            }
        }
    }

    model: Mapit.RootTreeModel {
        sortRole: 0 // "displayRole"
        id: rootModel
        root: currentWorkspace
        function forEachItem(parentItem, callback) {
            //console.log("DBG: a" + hasIndex(0, Mapit.RootTreeModel.NodeTypeRole))
            for(var i = 0; hasIndex(i, 0, parentItem); ++i) {
                var itemIndex = index(i, 0, parentItem)
                //console.log("DBG: b" + itemIndex)
                callback(itemIndex, i, parentItem)
                if(hasChildren(itemIndex))
                    forEachItem(itemIndex, callback, parentItem)
            }
        }

        onItemsChanged: {
            var newItem = []
            var missingItems = []
            for(var missingIndex=0 ; missingIndex < treeViewWorkspace.allVisualInfoModel.length ; ++missingIndex) {
                missingItems.push(treeViewWorkspace.allVisualInfoModel[missingIndex])
            }

            forEachItem(null, function(itemIndex, i, parentItem) {
                var itemPath = data(itemIndex, Mapit.RootTreeModel.NodePathRole)
                var itemIsEntity = data(itemIndex, Mapit.RootTreeModel.NodeTypeRole) === Mapit.RootTreeModel.EntityNode
                var foundItem = treeViewWorkspace.getVisualInfoForPath(itemPath, itemIsEntity)
                //setData(itemIndex, foundItem, Mapit.RootTreeModel.NodeVisualInfoRole)
                for(var missingIndex=0 ; missingIndex < missingItems.length ; ++missingIndex) {
                    if(missingItems[missingIndex].path === foundItem.path) {
                        missingItems.splice(missingIndex, 1)
                        break
                    }
                }
            })
            var infoModelIndex = treeViewWorkspace.allVisualInfoModel.length
            while(infoModelIndex--) {
                for(var missingIndex=0 ; missingIndex < missingItems.length ; ++missingIndex) {
                    if(missingItems[missingIndex].path === treeViewWorkspace.allVisualInfoModel[infoModelIndex].path) {
                        treeViewWorkspace.allVisualInfoModel[infoModelIndex].isVisible = false
                        // do not remove old visual info but hide them and keep them cached
                        //treeViewWorkspace.allVisualInfoModel.remove(infoModelIndex)
                        break
                    }
                }
            }
        }
    }
    QCtl.TableViewColumn {
        role: "displayRole"
        title: "Name"
        resizable: true
        width: treeViewWorkspace.width-appStyle.controlHeightInner-visibleColumn.width-12 /*12=scrollbarwidth*/
        delegate: StyledLabel {
            id: itemLabel
            height: appStyle.controlHeightInner
            verticalAlignment:  Text.AlignVCenter
            text: styleData.value
            elide: StyledLabel.ElideRight
            property var entity: currentWorkspace.getEntity(rootModel.data(styleData.index, Mapit.RootTreeModel.NodePathRole))
            tooltip: entity && entity.isValid() ?
                                ("<b>Type:</b> " + entity.type
                           + "<br><b>Frame:</b> " + entity.frameId
                           + "<br><b>Stamp:</b> " + entity.stamp.text):""

            property string currentClassName: "MapitEntity"
            property var parameters: { "currentEntityPath": rootModel.data(styleData.index, Mapit.RootTreeModel.NodePathRole) }
            property var myGraph
            property bool myIsClass: true
            property bool dragActive: dragArea.drag.active
            property real dragStartX
            property real dragStartY
            onDragActiveChanged: {
                forceActiveFocus();
                if (dragActive) {
                    dragStartX = x;
                    dragStartY = y;
                    Drag.start();
                } else {
                    Drag.drop();
                    x = dragStartX;
                    y = dragStartY;
                }
            }
            Drag.source: itemLabel
            Drag.dragType: Drag.Internal
            MouseArea {
                id: dragArea
                anchors.fill: parent
                drag.target: parent
                onClicked: treeViewWorkspace.selection.setCurrentIndex(styleData.index, ItemSelectionModel.ClearAndSelect)
            }

        }
    }
//    QCtl.TableViewColumn {
//        id: pathColumn
//        role: "path"
//        title: "Path"
//    }
    Component {
        id: visualInfoComponent
        EntityVisualInfo {}
    }

    itemDelegate: Component {
        Row {
            property EntityVisualInfo myVisualInfo: treeViewWorkspace.getVisualInfoForPath(rootModel.data(styleData.index, Mapit.RootTreeModel.NodePathRole),
                                                                             rootModel.data(styleData.index, Mapit.RootTreeModel.NodeTypeRole) === Mapit.RootTreeModel.EntityNode)

            padding: appStyle.controlMargin
            width: visibleColumn.width
            height: appStyle.controlHeightInner
            QCtl.BusyIndicator {
                anchors.top: parent.top
                anchors.bottom: parent.bottom
                width: height
                running: myVisualInfo.isLoading
            }
            MouseArea {
                id: itemMA
                anchors.top: parent.top
                anchors.bottom: parent.bottom
                width: height
                Image {
                    id: visibleImage
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    fillMode: Image.PreserveAspectFit
                    source: "image://icon/eye"
                    sourceSize: Qt.size(appStyle.iconSize, appStyle.iconSize)
                    visible: false
                }
                ColorOverlay {
                    anchors.fill: visibleImage
                    source: visibleImage
                    color: myVisualInfo ? myVisualInfo.isVisible ? Qt.rgba(0,0,0,1.0) : Qt.rgba(0.5,0.5,0.5,1.0) : "red"
                    visible: true
                }
                onClicked: {
                    myVisualInfo.isVisible = !myVisualInfo.isVisible
                    if(!myVisualInfo.isEntity) {
                        // tree
                        rootModel.forEachItem(styleData.index, function(itemIndexI, i2, parentItem) {
                            var path = rootModel.data(itemIndexI, Mapit.RootTreeModel.NodePathRole)
                            var itemIsEntity = rootModel.data(itemIndexI, Mapit.RootTreeModel.NodeTypeRole) === Mapit.RootTreeModel.EntityNode
                            var visInfo = treeViewWorkspace.getVisualInfoForPath(path, itemIsEntity)
                            visInfo.isVisible = myVisualInfo.isVisible
                        })
                    }
                }
            }
        }
    }

    QCtl.TableViewColumn {
        id: visibleColumn
        role: "visible"
        title: "Vis"
        movable: false
        resizable: false
        width: appStyle.controlHeightInner*2

    }
    rowDelegate: Component {
            Rectangle {
                height: appStyle.controlHeightInner
//                anchors.left: parent !== null ? parent.left : warningSuppressor.left
//                anchors.right: parent !== null ? parent.right : warningSuppressor.right
//                anchors.verticalCenter: parent !== null ? parent.verticalCenter : warningSuppressor.verticalCenter
//                height: parent !== null ? parent.height : warningSuppressor.height
                color: styleData.selected ? appStyle.highlightColor : appStyle.itemBackgroundColor
//TODO: Knwon Bug: When visible entities are hidden while list is shared over network, qml/ObjectModel/ListModel crashes.
                MouseArea {
                    anchors.fill: parent
                    acceptedButtons: Qt.RightButton
                    onClicked: {
                        if (mouse.button === Qt.RightButton) {
                            var mouseCoords = mapToItem(treeViewWorkspace, mouseX, mouseY)
                            treeViewWorkspace.contextMenu.index = treeViewWorkspace.indexAt(mouseCoords.x, mouseCoords.y)
                            treeViewWorkspace.contextMenu.path = rootModel.data(treeViewWorkspace.contextMenu.index, Mapit.RootTreeModel.NodePathRole)
                            treeViewWorkspace.contextMenu.itemIsEntity =rootModel.data(treeViewWorkspace.contextMenu.index, Mapit.RootTreeModel.NodeTypeRole) === Mapit.RootTreeModel.EntityNode
                            console.log("DBG: PATH: " + treeViewWorkspace.contextMenu.path)
                            treeViewWorkspace.contextMenu.popup()
                        }
                    }
                }
            }
    }
}
