import QtQuick 2.4
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1

import fhac.upns 1.0 as UPNS

QCtl.TreeView {
    property var currentCheckout
    property var contextMenu
    id: treeViewCheckout
    model: UPNS.RootTreeModel {
        root: currentCheckout
    }
    QCtl.TableViewColumn {
        role: "displayRole"
        title: "Name"
    }
    QCtl.TableViewColumn {
        id: pathColumn
        role: "path"
        title: "Path"
    }
    onCurrentIndexChanged: console.log(treeViewCheckout.model.data(treeViewCheckout.currentIndex, UPNS.RootTreeModel.NodeTypeRole));//treeViewCheckout.currentIndex.data(Qt.ToolTipRole));//treeViewCheckout.model.data(treeViewCheckout.currentIndex, Qt.ToolTipRole))
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
