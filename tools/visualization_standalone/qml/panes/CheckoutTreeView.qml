import QtQuick 2.7
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtGraphicalEffects 1.0
import QtQml.Models 2.3

import fhac.upns 1.0 as UPNS

import "../components"
import "../network"

QCtl.TreeView {
    id: treeViewCheckout
    property var currentCheckout: globalApplicationState.currentCheckout
    property var contextMenu
    property var visibleElems: [] //ListModel {}
    property var visualInfoModel: ObjectModel {}
    property var visibleEntityModel: ListModel {}
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
        for(var i=0 ; visualInfoModel.count > i ; ++i) {
            var current = visualInfoModel.get(i)
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
                    treeViewCheckout.visibleEntityModel.append({path:newObject.path})
                }
            } else {
                for(var i=0 ; i < treeViewCheckout.visibleEntityModel.count ; ++i) {
                    if(treeViewCheckout.visibleEntityModel.get(i).path === newObject.path) {
                        console.log("DBG: removed: " + newObject.path)
                        treeViewCheckout.visibleEntityModel.remove(newObject)
                    }
                }
            }
        })
        visualInfoModel.append(newObject)
        return newObject
    }

    model: UPNS.RootTreeModel {
        sortRole: 0 // "displayRole"
        id: rootModel
//        Component.onCompleted: {
//            visiblePathToVisibility = {}
//        }
//        property var visiblePathToVisibility
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
            for(var missingIndex=0 ; missingIndex < treeViewCheckout.visualInfoModel.count ; ++missingIndex) {
                missingItems.push(treeViewCheckout.visualInfoModel.get(missingIndex))
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
            var infoModelIndex = treeViewCheckout.visualInfoModel.count
            while(infoModelIndex--) {
                for(var missingIndex=0 ; missingIndex < missingItems.length ; ++missingIndex) {
                    if(missingItems[missingIndex].path === treeViewCheckout.visualInfoModel.get(infoModelIndex).path) {
                        treeViewCheckout.visualInfoModel.get(infoModelIndex).isVisible = false
                        // do not remove old visual info but hide them and keep them cached
                        //treeViewCheckout.visualInfoModel.remove(infoModelIndex)
                        break
                    }
                }
            }

////            forEachItem(null, function(itemIndex, i, parentItem) {
////                var itemPath = data(itemIndex, UPNS.RootTreeModel.NodePathRole)
////                console.log("DBG: path: " + itemPath)
////                var newVisibleItems = []
////                var found = false
////                for (var i=0; i < treeViewCheckout.visibleElems.length; i++) {
////                    if(treeViewCheckout.visibleElems[i].path === itemPath) {
////                        found = true
////                        break;
////                    }
////                }
////                for (var visiblePath in rootModel.visiblePathToVisibility)
////                {
////                    if(visiblePath === itemPath) {
////                        rootModel.visiblePathToVisibility[itemPath] = found
////                        if( !found )
////                            rootModel.visiblePathToVisibility.remove(itemPath)
////                        break;
////                    }
////                }
////            })

////            for(var i=0 ; i < treeViewCheckout.visibleElems.length ; ++i) {
////                var propertyname
////                var obj = treeViewCheckout.visibleElems[i];
////                if(obj.path === styleData.row) {
////                    treeViewCheckout.visibleElems.splice(i,1);
////                }
////            }
//            //TODO: rebuild list!
//            //console.log("DBG: CALLED: " + treeViewCheckout.visibleElems.length)
//            var i=treeViewCheckout.visibleElems.length
//            while (i--) {
//                console.log("DBG: arr:" + i)
//                var found = false
//                forEachItem(null, function(itemIndex, i2, parentItem) {
//                    var itemPath = data(itemIndex, UPNS.RootTreeModel.NodePathRole)
//                    console.log("DBG: i:" + itemPath + "  " + treeViewCheckout.visibleElems[i].path)
//                    if(treeViewCheckout.visibleElems[i].path === itemPath) {
//                        found = true
//                    }
//                })
//                if( !found ) {
//                    console.log("DBG: REMOVED: " + i)
//                    treeViewCheckout.visibleElems.splice(i,1)
//                }
//            }
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
            property var entity: currentCheckout.getEntity(model.path)
            tooltip: entity && entity.isValid() ?
                                ("<b>Type:</b> " + entity.type
                           + "<br><b>Frame:</b> " + entity.frameId
                           + "<br><b>Stamp:</b> " + entity.stamp.text):""

            property string currentClassName: "MapitEntity"
            property var parameters: { "currentEntityPath": model.path }
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
            property var myVisualInfo: treeViewCheckout.getVisualInfoForPath(rootModel.data(styleData.index, UPNS.RootTreeModel.NodePathRole),
                                                                             rootModel.data(styleData.index, UPNS.RootTreeModel.NodeTypeRole) === UPNS.RootTreeModel.EntityNode)

            padding: appStyle.controlMargin
            width: visibleColumn.width
            height: appStyle.controlHeightInner
            QCtl.BusyIndicator {
                anchors.top: parent.top
                anchors.bottom: parent.bottom
                //Layout.fillHeight: true
//                height: appStyle.controlHeightInner
                width: height
                running: myVisualInfo.isLoading
            }
            MouseArea {
                id: itemMA
                anchors.top: parent.top
                anchors.bottom: parent.bottom
                //Layout.fillHeight: true
//                height: appStyle.controlHeightInner
                width: height
                Image {
                    id: visibleImage
                    anchors.top: parent.top
                    anchors.bottom: parent.bottom
                    fillMode: Image.PreserveAspectFit
                    //anchors.verticalCenter: parent.verticalCenter
                    source: "image://icon/eye"
                    sourceSize: Qt.size(appStyle.iconSize, appStyle.iconSize)
                    visible: false
                }
                ColorOverlay {
                    anchors.fill: visibleImage
                    source: visibleImage
                    color: myVisualInfo ? myVisualInfo.isVisible ? Qt.rgba(0,0,0,1.0) : Qt.rgba(0.5,0.5,0.5,1.0) : "red"
//                               {
//                        for(var i=0 ; i < treeViewCheckout.visibleElems.length ; ++i) {
//                            var obj = treeViewCheckout.visibleElems[i];
//                            console.log("DBG:eq: " + obj.path + " === " + rootModel.data(styleData.index, UPNS.RootTreeModel.NodePathRole))
//                            if(obj.path === rootModel.data(styleData.index, UPNS.RootTreeModel.NodePathRole)) {
//                                return Qt.rgba(0,0,0,1.0)
//                            }
//                        }
//                        return Qt.rgba(0.5,0.5,0.5,1.0) // TODO: style
//                        //itemMA.showObj ? "#43adee" : "#565656"
//                    }
                    visible: true // model ? model.type : false
                }
                onClicked: {
                    myVisualInfo.isVisible = !myVisualInfo.isVisible

//                    var modelType = rootModel.data(styleData.index, UPNS.RootTreeModel.NodeTypeRole)
//                    var newVisibilityValue = rootModel.data(styleData.index, UPNS.RootTreeModel.NodeVisibleRole)
//                    rootModel.setData(styleData.index, !newVisibilityValue, UPNS.RootTreeModel.NodeVisibleRole)
                    if(!myVisualInfo.isEntity) {
                        // tree
                        //var theIndex = styleData.index //rootModel.index(styleData.row, styleData.column)
                        //console.log("DBG: ELEM: " + styleData.row + ", " + styleData.column + " i:" + theIndex + ", path: " + rootModel.data(theIndex, UPNS.RootTreeModel.NodePathRole) + ", data: " + rootModel.data(theIndex) + " id: " + theIndex.data + " o " + theIndex.data)
                        rootModel.forEachItem(styleData.index, function(itemIndexI, i2, parentItem) {
                            //rootModel.setData(itemIndexI, newVisibilityValue, UPNS.RootTreeModel.NodeVisibleRole)
                            var path = rootModel.data(itemIndexI, UPNS.RootTreeModel.NodePathRole)
                            var itemIsEntity = rootModel.data(itemIndexI, UPNS.RootTreeModel.NodeTypeRole) === UPNS.RootTreeModel.EntityNode
                            var visInfo = treeViewCheckout.getVisualInfoForPath(path, itemIsEntity)
                            visInfo.isVisible = myVisualInfo.isVisible
//                            if(!myVisualInfo.isVisible) {
//                                for(var i=0 ; i < treeViewCheckout.myVisualInfo.length ; ++i) {
//                                    var visInfo = treeViewCheckout.myVisualInfo.get(i);
//                                    console.log("DBG: idxT: " + (typeof visInfo) + ", val: " + visInfo.path + " == " + itemIndexI)
//                                    if(visInfo.path === rootModel.data(itemIndexI, UPNS.RootTreeModel.NodePathRole)) {
//                                        visInfo.isVisible = myVisualInfo.isVisible
//                                    }
//                                }
//                                globalApplicationState.visibleEntityPaths = treeViewCheckout.visibleElems
//                            } else {
//                                treeViewCheckout.visibleElems.push({idx:itemIndexI, path:rootModel.data(itemIndexI, UPNS.RootTreeModel.NodePathRole), checkoutName: currentCheckout.name})
//                                globalApplicationState.visibleEntityPaths = treeViewCheckout.visibleElems
//                            }
                        })
                    } //else {
//                        if(!newVisibilityValue) {
//                            for(var i=0 ; i < treeViewCheckout.visibleElems.length ; ++i) {
//                                var obj = treeViewCheckout.visibleElems[i];
//                                if(obj.path === rootModel.data(styleData.index, UPNS.RootTreeModel.NodePathRole)) {
//                                    treeViewCheckout.visibleElems.splice(i,1);
//                                }
//                            }
//                            globalApplicationState.visibleEntityPaths = treeViewCheckout.visibleElems
//                        } else {
//                            treeViewCheckout.visibleElems.push({idx:styleData.index, path:rootModel.data(styleData.index, UPNS.RootTreeModel.NodePathRole), checkoutName: currentCheckout.name})
//                            globalApplicationState.visibleEntityPaths = treeViewCheckout.visibleElems
//                        }
//                    }
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
//        delegate: MouseArea {
//            id: itemMA
//            Image {
//                id: visibleImage
//                anchors.top: parent.top
//                anchors.bottom: parent.bottom
//                fillMode: Image.PreserveAspectFit
//                //anchors.verticalCenter: parent.verticalCenter
//                source: "image://icon/eye"
//                sourceSize: Qt.size(appStyle.iconSize, appStyle.iconSize)
//                visible: false
//            }
//            ColorOverlay {
//                anchors.fill: visibleImage
//                source: visibleImage
//                color: {
//                    for(var i=0 ; i < treeViewCheckout.visibleElems.length ; ++i) {
//                        var obj = treeViewCheckout.visibleElems[i];
//                        if(obj.idx === styleData.row) {
//                            return Qt.rgba(0,0,0,1.0)
//                        }
//                    }
//                    return Qt.rgba(0.5,0.5,0.5,1.0) // TODO: style
//                    //itemMA.showObj ? "#43adee" : "#565656"
//                }
//                visible: true // model ? model.type : false
//            }
//            onClicked: {
//                model.visible = !model.visible
//                if(!model.type) {
//                    // tree
//                    var theIndex = rootModel.index(styleData.row, styleData.column)
//                    console.log("DBG: ELEM: " + styleData.row + ", " + styleData.column + " i:" + theIndex + ", path: " + rootModel.data(theIndex, UPNS.RootTreeModel.NodePathRole) + ", data: " + rootModel.data(theIndex) + " id: " + theIndex.data + " o " + theIndex.data)
//                    rootModel.forEachItem(theIndex, function(itemIndexI, i2, parentItem) {
//                        rootModel.setData(itemIndexI, model.visible, UPNS.RootTreeModel.NodeVisibleRole)
//                        if(!model.visible) {
//                            for(var i=0 ; i < treeViewCheckout.visibleElems.length ; ++i) {
//                                var obj = treeViewCheckout.visibleElems[i];
//                                if(obj.idx === i2) {
//                                    treeViewCheckout.visibleElems.splice(i,1);
//                                }
//                            }
//                        } else {
//                            treeViewCheckout.visibleElems.push({idx:i2, path:rootModel.setData(itemIndexI, model.visible, UPNS.RootTreeModel.NodePathRole), checkoutName: currentCheckout.name})
//                            globalApplicationState.visibleEntityPaths = treeViewCheckout.visibleElems
//                        }
//                    })
//                } else {
//                    if(!styleData.value) {
//                        for(var i=0 ; i < treeViewCheckout.visibleElems.length ; ++i) {
//                            var obj = treeViewCheckout.visibleElems[i];
//                            if(obj.idx === styleData.row) {
//                                treeViewCheckout.visibleElems.splice(i,1);
//                            }
//                        }
//                    } else {
//                        treeViewCheckout.visibleElems.push({idx:styleData.row, path:model.path, checkoutName: currentCheckout.name})
//                        globalApplicationState.visibleEntityPaths = treeViewCheckout.visibleElems
//                    }
//                }
//            }
//        }
//            QCtl.CheckBox {
//            enabled: model.type
//            onCheckedChanged: {
//                if(!checked) {
//                    for(var i=0 ; i < treeViewCheckout.visibleElems.length ; ++i) {
//                        var obj = treeViewCheckout.visibleElems[i];
//                        if(obj.idx === styleData.row) {
//                            treeViewCheckout.visibleElems.splice(i,1);
//                        }
//                    }
//                } else {
//                    treeViewCheckout.visibleElems.push({idx:styleData.row, path:model.path, checkoutName: currentCheckout.name})
//                }
//            }
//        }
    }
    //onCurrentIndexChanged: console.log("SEL:"+treeViewCheckout.model.data(treeViewCheckout.currentIndex, UPNS.RootTreeModel.NodeTypeRole));//treeViewCheckout.currentIndex.data(Qt.ToolTipRole));//treeViewCheckout.model.data(treeViewCheckout.currentIndex, Qt.ToolTipRole))
    rowDelegate: Component {
            Rectangle {
                height: appStyle.controlHeightInner
//                anchors.left: parent !== null ? parent.left : warningSuppressor.left
//                anchors.right: parent !== null ? parent.right : warningSuppressor.right
//                anchors.verticalCenter: parent !== null ? parent.verticalCenter : warningSuppressor.verticalCenter
//                height: parent !== null ? parent.height : warningSuppressor.height
                color: styleData.selected ? appStyle.highlightColor : appStyle.itemBackgroundColor

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
