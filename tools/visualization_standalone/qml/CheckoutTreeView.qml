import QtQuick 2.4
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtGraphicalEffects 1.0

import fhac.upns 1.0 as UPNS

QCtl.TreeView {
    property var currentCheckout
    property var contextMenu
    property var visibleElems: ListModel {}
    alternatingRowColors: true
    headerVisible: false
    id: treeViewCheckout
    model: UPNS.RootTreeModel {
//        property var visiblePathToVisibility: {}
        root: currentCheckout
//        function forEachItem(parentItem, callback) {
//            for(var i = 0; hasIndex(i, 0, parentItem); ++i) {
//                var itemIndex = index(i, 0, parentItem)
//                callback(itemIndex, i, parentItem)
//                if(hasChildren(itemIndex))
//                    forEachItem(itemIndex, callback, parentItem)
//            }
//        }

//        onDataChanged: {
//            forEachItem(null, function(itemIndex, i, parentItem) {
//                var itemPath = data(itemIndex, UPNS.RootTreeModel.NodePathRole)
//                console.log("DBG: path: " + itemPath)
//                var newVisibleItems = []
//                var found = false
//                for (var i=0; i < treeViewCheckout.visibleElems.count; i++)
//                    if(treeViewCheckout.visibleElems.get(i).path === itemPath) {
//                        found = true
//                        continue;
//                    }
//                }
//                for (var visiblePath in visiblePathToVisibility)
//                {
//                    if(visiblePath === itemPath) {
//                        newVisibleItems.push()
//                        continue;
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
//        }
    }
    QCtl.TableViewColumn {
        role: "displayRole"
        title: "Name"
        resizable: true
        width: treeViewCheckout.width-30-4
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
        width: 30
        delegate: MouseArea {
            id: itemMA
            property bool showObj: false
            Image {
                id: visibleImage
                anchors.top: parent.top
                anchors.bottom: parent.bottom
                fillMode: Image.PreserveAspectFit
                //anchors.verticalCenter: parent.verticalCenter
                source: "image://icon/eye"
                visible: model?model.type:false
            }
            ColorOverlay {
                anchors.fill: visibleImage
                source: visibleImage
                color: {
                    for(var i=0 ; i < treeViewCheckout.visibleElems.count ; ++i) {
                        var obj = treeViewCheckout.visibleElems.get(i);
                        if(obj.idx === styleData.row) {
                            return "#43adee"
                        }
                    }
                    return "#565656"
                    //itemMA.showObj ? "#43adee" : "#565656"
                }
                visible: model?model.type:false
            }
            onClicked: {
                if(!model.type) return
                itemMA.showObj = !itemMA.showObj
                if(!itemMA.showObj) {
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
        Rectangle {
            anchors {
                left: parent.left
                right: parent.right
                verticalCenter: parent.verticalCenter
            }
            height: parent.height
            color: styleData.selected ? palette.highlight : palette.base
            MouseArea {
                anchors.fill: parent
                acceptedButtons: Qt.RightButton
                onClicked: {
                    if (mouse.button === Qt.RightButton)
                    {
                        treeViewCheckout.contextMenu.popup()
                    }
                }
            }
        }
    }
}
