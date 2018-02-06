import QtQuick 2.7
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtGraphicalEffects 1.0
import QtQml.Models 2.3

import fhac.upns 1.0 as UPNS

import "../components"
import "qrc:/qml/network"

QCtl.TreeView {
    id: treeViewCheckout
    property var currentCheckout: globalApplicationState.currentCheckout
    property var contextMenu
    // all objects, also cached and invisible
    property var allVisualInfoModel: ([])//ObjectModel {}
    // only visible entities
    property ListModel visibleEntityModel: ListModel {}
    property string currentEntityPath: selectionModel.currentIndex && treeViewCheckout.model.data(selectionModel.currentIndex, UPNS.RootTreeModel.NodeTypeRole) === UPNS.RootTreeModel.EntityNode ? treeViewCheckout.model.data(selectionModel.currentIndex, Qt.ToolTipRole) : ""
    signal visibleElemsUpdated
    alternatingRowColors: true
    headerVisible: false
    backgroundVisible: false
    //selectionMode: SelectionMode.SingleSelection
    selection: ItemSelectionModel {
        id: selectionModel
        model: treeViewCheckout.model
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
        for(var i=0 ; treeViewCheckout.allVisualInfoModel.length > i ; ++i) {
            var current = treeViewCheckout.allVisualInfoModel[i]
            if(current.path === path) {
                current.isEntity = isEntity
                return current
            }
        }
        var newObject = visualInfoComponent.createObject(treeViewCheckout)
        newObject.path = path
        newObject.isEntity = isEntity
        newObject.onIsVisibleChanged.connect(function(isVisible) {
            if(!newObject.isEntity) return
            if(newObject.isVisible) {
                var found = false
                for(var i=0 ; i < treeViewCheckout.visibleEntityModel.count ; ++i) {
                    if(treeViewCheckout.visibleEntityModel.get(i).path === newObject.path) {
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
                    for(var i=0 ; i < treeViewCheckout.allVisualInfoModel.length ; ++i) {
                        if(treeViewCheckout.allVisualInfoModel[i].path === newObject.path) {
                            idx = i
                            break
                        }
                    }
                    listSynchronizer.itemsToAdd.push({path:newObject.path, idxInVisualInfoModel:idx})
                    listSynchronizer.start()
//                    treeViewCheckout.visibleEntityModel.append({path:newObject.path, idxInVisualInfoModel:idx})
//                    var arr = treeViewCheckout.visibleEntityModel
//                    arr.push({path:newObject.path, idxInVisualInfoModel:idx})
//                    treeViewCheckout.visibleEntityModel = arr
                }
            } else {
                listSynchronizer.itemsToRemove.push(newObject)
                listSynchronizer.start()
//                console.log("DBG: getVisualInfoForPath: removing count: " + treeViewCheckout.visibleEntityModel.count)
//                for(var i=0 ; i < treeViewCheckout.visibleEntityModel.count ; ++i) {
//                    console.log("DBG: getVisualInfoForPath: treeViewCheckout.visibleEntityModel index: " + i)
//                    if(treeViewCheckout.visibleEntityModel.get(i).path === newObject.path) {
//                        console.log("DBG: getVisualInfoForPath: removed newObject.path: " + newObject.path)
//                        treeViewCheckout.visibleEntityModel.remove(i)
//                        console.log("DBG: getVisualInfoForPath: deleted newObject.path: " + newObject.path)
////                        var arr = treeViewCheckout.visibleEntityModel
////                        arr.splice(i, 1)
////                        treeViewCheckout.visibleEntityModel = arr
//                    }
//                }
            }
        })
        listSynchronizer.itemsToAddToAllVisualInfoModel.push(newObject)
        listSynchronizer.start()
//        treeViewCheckout.allVisualInfoModel.push(newObject)
        return newObject
    }
    Timer {
        // it is not safe to call treeViewCheckout.visibleEntityModel.remove(i) while view are processed.
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
                for(var i=0 ; i < treeViewCheckout.visibleEntityModel.count ; ++i) {
                    if(treeViewCheckout.visibleEntityModel.get(i).path === removeItem.path) {
                        treeViewCheckout.visibleEntityModel.remove(i)
                    }
                }
                listSynchronizer.itemsToRemove.splice(toRemoveIndex, 1)
            }
            var toAddIndex = listSynchronizer.itemsToAdd.length
            while(toAddIndex--) {
                var addItem = listSynchronizer.itemsToAdd[toAddIndex]
                treeViewCheckout.visibleEntityModel.append(addItem)
                listSynchronizer.itemsToAdd.splice(toAddIndex, 1)
            }
            var toAddIndex2 = listSynchronizer.itemsToAddToAllVisualInfoModel.length
            while(toAddIndex2--) {
                var addItem2 = listSynchronizer.itemsToAddToAllVisualInfoModel[toAddIndex2]
                treeViewCheckout.allVisualInfoModel.push(addItem2)
                listSynchronizer.itemsToAddToAllVisualInfoModel.splice(toAddIndex2, 1)
            }
        }
    }

    model: UPNS.RootTreeModel {
        sortRole: 0 // "displayRole"
        id: rootModel
        root: currentCheckout
        function forEachItem(parentItem, callback) {
            //console.log("DBG: a" + hasIndex(0, UPNS.RootTreeModel.NodeTypeRole))
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
            for(var missingIndex=0 ; missingIndex < treeViewCheckout.allVisualInfoModel.length ; ++missingIndex) {
                missingItems.push(treeViewCheckout.allVisualInfoModel[missingIndex])
            }

            forEachItem(null, function(itemIndex, i, parentItem) {
                var itemPath = data(itemIndex, UPNS.RootTreeModel.NodePathRole)
                var itemIsEntity = data(itemIndex, UPNS.RootTreeModel.NodeTypeRole) === UPNS.RootTreeModel.EntityNode
                var foundItem = treeViewCheckout.getVisualInfoForPath(itemPath, itemIsEntity)
                //setData(itemIndex, foundItem, UPNS.RootTreeModel.NodeVisualInfoRole)
                for(var missingIndex=0 ; missingIndex < missingItems.length ; ++missingIndex) {
                    if(missingItems[missingIndex].path === foundItem.path) {
                        missingItems.splice(missingIndex, 1)
                        break
                    }
                }
            })
            var infoModelIndex = treeViewCheckout.allVisualInfoModel.length
            while(infoModelIndex--) {
                for(var missingIndex=0 ; missingIndex < missingItems.length ; ++missingIndex) {
                    if(missingItems[missingIndex].path === treeViewCheckout.allVisualInfoModel[infoModelIndex].path) {
                        treeViewCheckout.allVisualInfoModel[infoModelIndex].isVisible = false
                        // do not remove old visual info but hide them and keep them cached
                        //treeViewCheckout.allVisualInfoModel.remove(infoModelIndex)
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
        width: treeViewCheckout.width-appStyle.controlHeightInner-visibleColumn.width-12 /*12=scrollbarwidth*/
        delegate: StyledLabel {
            id: itemLabel
            height: appStyle.controlHeightInner
            verticalAlignment:  Text.AlignVCenter
            text: styleData.value
            elide: StyledLabel.ElideRight
            property var entity: currentCheckout.getEntity(rootModel.data(styleData.index, UPNS.RootTreeModel.NodePathRole))
            tooltip: entity && entity.isValid() ?
                                ("<b>Type:</b> " + entity.type
                           + "<br><b>Frame:</b> " + entity.frameId
                           + "<br><b>Stamp:</b> " + entity.stamp.text):""

            property string currentClassName: "MapitEntity"
            property var parameters: { "currentEntityPath": rootModel.data(styleData.index, UPNS.RootTreeModel.NodePathRole) }
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
                onClicked: treeViewCheckout.selection.setCurrentIndex(styleData.index, ItemSelectionModel.ClearAndSelect)
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
            property EntityVisualInfo myVisualInfo: treeViewCheckout.getVisualInfoForPath(rootModel.data(styleData.index, UPNS.RootTreeModel.NodePathRole),
                                                                             rootModel.data(styleData.index, UPNS.RootTreeModel.NodeTypeRole) === UPNS.RootTreeModel.EntityNode)

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
                            var path = rootModel.data(itemIndexI, UPNS.RootTreeModel.NodePathRole)
                            var itemIsEntity = rootModel.data(itemIndexI, UPNS.RootTreeModel.NodeTypeRole) === UPNS.RootTreeModel.EntityNode
                            var visInfo = treeViewCheckout.getVisualInfoForPath(path, itemIsEntity)
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
                            treeViewCheckout.contextMenu.popup()
                        }
                    }
                }
            }
    }
}
