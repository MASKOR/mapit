import QtQuick 2.4
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtGraphicalEffects 1.0

import fhac.upns 1.0 as UPNS

QCtl.TreeView {
    id: treeViewCheckout
    property var currentCheckout
    property var contextMenu
    property var visibleElems: ListModel {}
    alternatingRowColors: true
    headerVisible: false
    backgroundVisible: false
    //selectionMode: SelectionMode.SingleSelection
    model: UPNS.RootTreeModel {
        id: rootModel
//        Component.onCompleted: {
//            visiblePathToVisibility = {}
//        }
//        property var visiblePathToVisibility
        root: currentCheckout
        function forEachItem(parentItem, callback) {
            console.log("DBG: a" + hasIndex(0, UPNS.RootTreeModel.NodeTypeRole))
            for(var i = 0; hasIndex(i, 0, parentItem); ++i) {
                var itemIndex = index(i, 0, parentItem)
                console.log("DBG: b" + itemIndex)
                callback(itemIndex, i, parentItem)
                if(hasChildren(itemIndex))
                    forEachItem(itemIndex, callback, parentItem)
            }
        }

        onItemsChanged: {
//            forEachItem(null, function(itemIndex, i, parentItem) {
//                var itemPath = data(itemIndex, UPNS.RootTreeModel.NodePathRole)
//                console.log("DBG: path: " + itemPath)
//                var newVisibleItems = []
//                var found = false
//                for (var i=0; i < treeViewCheckout.visibleElems.count; i++) {
//                    if(treeViewCheckout.visibleElems.get(i).path === itemPath) {
//                        found = true
//                        break;
//                    }
//                }
//                for (var visiblePath in rootModel.visiblePathToVisibility)
//                {
//                    if(visiblePath === itemPath) {
//                        rootModel.visiblePathToVisibility[itemPath] = found
//                        if( !found )
//                            rootModel.visiblePathToVisibility.remove(itemPath)
//                        break;
//                    }
//                }
//            })

//            for(var i=0 ; i < treeViewCheckout.visibleElems.count ; ++i) {
//                var propertyname
//                var obj = treeViewCheckout.visibleElems.get(i);
//                if(obj.path === styleData.row) {
//                    treeViewCheckout.visibleElems.remove(i);
//                }
//            }
            //TODO: rebuild list!
            //console.log("DBG: CALLED: " + treeViewCheckout.visibleElems.count)
            var i=treeViewCheckout.visibleElems.count
            while (i--) {
                console.log("DBG: arr:" + i)
                var found = false
                forEachItem(null, function(itemIndex, i2, parentItem) {
                    var itemPath = data(itemIndex, UPNS.RootTreeModel.NodePathRole)
                    console.log("DBG: i:" + itemPath + "  " + treeViewCheckout.visibleElems.get(i).path)
                    if(treeViewCheckout.visibleElems.get(i).path === itemPath) {
                        found = true
                    }
                })
                if( !found ) {
                    console.log("DBG: REMOVED: " + i)
                    treeViewCheckout.visibleElems.remove(i)
                }
            }
        }
    }
    QCtl.TableViewColumn {
        role: "displayRole"
        title: "Name"
        resizable: true
        width: treeViewCheckout.width-appStyle.controlHeightInner-visibleColumn.width
        delegate: StyledLabel {
            height: appStyle.controlHeightInner
            verticalAlignment:  Text.AlignVCenter
            text: styleData.value
        }
    }
//    QCtl.TableViewColumn {
//        id: pathColumn
//        role: "path"
//        title: "Path"
//    }
    QCtl.TableViewColumn {
        id: visibleColumn
        role: "visible"
        title: "Vis"
        movable: false
        resizable: false
        width: appStyle.controlHeightInner
        delegate: MouseArea {
            id: itemMA
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
                color: {
                    for(var i=0 ; i < treeViewCheckout.visibleElems.count ; ++i) {
                        var obj = treeViewCheckout.visibleElems.get(i);
                        if(obj.idx === styleData.row) {
                            return Qt.rgba(0,0,0,1.0)
                        }
                    }
                    return Qt.rgba(0.5,0.5,0.5,1.0) // TODO: style
                    //itemMA.showObj ? "#43adee" : "#565656"
                }
                visible: model ? model.type : false
            }
            onClicked: {
                if(!model.type) return
                model.visible = !model.visible
                if(!styleData.value) {
                    for(var i=0 ; i < treeViewCheckout.visibleElems.count ; ++i) {
                        var obj = treeViewCheckout.visibleElems.get(i);
                        if(obj.idx === styleData.row) {
                            treeViewCheckout.visibleElems.remove(i);
                        }
                    }
                } else {
                    treeViewCheckout.visibleElems.append({idx:styleData.row, path:model.path, checkoutName: currentCheckout.name})
                }
            }
        }
//            QCtl.CheckBox {
//            enabled: model.type
//            onCheckedChanged: {
//                if(!checked) {
//                    for(var i=0 ; i < treeViewCheckout.visibleElems.count ; ++i) {
//                        var obj = treeViewCheckout.visibleElems.get(i);
//                        if(obj.idx === styleData.row) {
//                            treeViewCheckout.visibleElems.remove(i);
//                        }
//                    }
//                } else {
//                    treeViewCheckout.visibleElems.append({idx:styleData.row, path:model.path, checkoutName: currentCheckout.name})
//                }
//            }
//        }
    }
    //onCurrentIndexChanged: console.log("SEL:"+treeViewCheckout.model.data(treeViewCheckout.currentIndex, UPNS.RootTreeModel.NodeTypeRole));//treeViewCheckout.currentIndex.data(Qt.ToolTipRole));//treeViewCheckout.model.data(treeViewCheckout.currentIndex, Qt.ToolTipRole))
    rowDelegate: Item {
        height: appStyle.controlHeightInner
        Rectangle {
            anchors {
                left: parent.left
                right: parent.right
                verticalCenter: parent.verticalCenter
            }
            height: parent.height
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
