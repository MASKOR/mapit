import QtQuick 2.4
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtQuick.Window 2.2 as Wnd
import QtGraphicalEffects 1.0
import pcl 1.0
import fhac.upns 1.0 as UPNS
import QtQuick 2.0 as QQ2
import "panes"

Item {
    id: root
    property alias currentOperator: operatorListPane.currentOperator
    property alias currentEntitydata: currentEntitydataId
    property alias currentEntitydataTransform: currentEntitydataTransformId
    property alias currentCheckout: checkout
    property string currentEntityPath: treeViewCheckout.currentIndex && treeViewCheckout.model.data(treeViewCheckout.currentIndex, UPNS.RootTreeModel.NodeTypeRole) === UPNS.RootTreeModel.EntityNode ? treeViewCheckout.model.data(treeViewCheckout.currentIndex, Qt.ToolTipRole) : ""
    UPNS.Checkout {
        id: checkout
        repository: globalRepository
        name: "testcheckout"
    }
    UPNS.Entitydata {
        id: currentEntitydataId
        checkout: checkout
        path: root.currentEntityPath// "corridor/lidar/pc1"
        onPathChanged: {
            console.log("Path of current entity: " + path)
        }
    }
    UPNS.EntitydataTransform {
        id: currentEntitydataTransformId
        checkout: checkout
        property bool addTfToPath: root.currentEntitydata.path.length > 3 && currentEntitydata.path.lastIndexOf(".tf") !== currentEntitydata.path.length-3
        path: currentEntitydata.path + (addTfToPath ? ".tf" : "")
        onPathChanged: console.log("New Path of Tf is: " + path)
    }
    ColumnLayout {
        anchors.fill: parent
        id: controlColumn
        Text {
            text: qsTr("Operators:")
            color: palette.text
        }
        OperatorListPane {
            Layout.fillWidth: true
            id: operatorListPane
            height: 200
        }

        QCtl.Button {
            text: "Checkout"
            onClicked: chooseCheckoutDialog.visible = !chooseCheckoutDialog.visible
            Wnd.Window {
                id: chooseCheckoutDialog
                width: 420
                height: 260
                minimumHeight: height
                maximumHeight: height
                minimumWidth: width
                maximumWidth: width
                flags: Qt.Dialog
                title: "Choose Checkout"
                color: palette.window
                ColumnLayout {
                    anchors.fill: parent

                    ListView {
                        id: checkoutList
                        delegate: RowLayout {
                            Image {
                                source: "image://icon/asset-green"
                            }
                                Text {
                                    text: globalRepository.checkoutNames[index]
                                    color: palette.text
                                    MouseArea {
                                        anchors.fill: parent
                                        onClicked: checkoutList.currentIndex = index
                                    }
                                }
                            }

                        model: globalRepository.checkoutNames
                        highlight: Rectangle { color: palette.highlight }

                        Layout.fillWidth: true
                        Layout.fillHeight: true
                    }
                    QCtl.Button {
                        text: "+"
                        onClicked: {
                            newCheckoutDialog.visible = !newCheckoutDialog.visible
                        }
                        Wnd.Window {
                            id: newCheckoutDialog
                            width: 420
                            height: 260
                            minimumHeight: height
                            maximumHeight: height
                            minimumWidth: width
                            maximumWidth: width
                            flags: Qt.Dialog
                            title: "Choose Checkout"
                            color: palette.window
                            GridLayout {
                                QCtl.Label {
                                    text: "Branchname"
                                    Layout.column: 0
                                    Layout.row: 0
                                }
                                QCtl.TextField {
                                    id: branchnameTextedit
                                    text: "master"
                                    Layout.column: 1
                                    Layout.row: 0
                                }
                                QCtl.Label {
                                    text: "Checkoutname"
                                    Layout.column: 0
                                    Layout.row: 1
                                }
                               QCtl. TextField {
                                    id: checkoutnameTextedit
                                    Layout.column: 1
                                    Layout.row: 1
                                }
                                QCtl.Button {
                                    text: "Cancel"
                                    onClicked: newCheckoutDialog.visible = false
                                    Layout.column: 0
                                    Layout.row: 2
                                }
                                QCtl.Button {
                                    text: "Ok"
                                    enabled: branchnameTextedit.text.trim().length !== 0
                                             && checkoutnameTextedit.text.trim().length !== 0
                                    onClicked: {
                                        globalRepository.createCheckout(branchnameTextedit.text, checkoutnameTextedit.text)
                                        newCheckoutDialog.visible = false
                                    }
                                    Layout.column: 1
                                    Layout.row: 2
                                }
                            }
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        QCtl.Button {
                            text: "Cancel"
                            onClicked: chooseCheckoutDialog.visible = false
                        }
                        QCtl.Button {
                            text: "Ok"
                            onClicked: {
                                checkout.name = globalRepository.checkoutNames[checkoutList.currentIndex];
                                chooseCheckoutDialog.visible = false;
                            }
                        }
                    }
                }
            }
        }
        QCtl.Action {
            id: transformAction
            text: "&Transform"
            shortcut: "T"
            //iconName: "edit-copy"
            enabled: currentEntitydataTransform.path.length > 0
            //    target: "path/to/target.tf"
            //    mode: "relative"|"absolute",
            //    tf: [
            //      0: {mat: [0: m00, 1: m01, m02...], parent: "/path/to/parent", timestamp: unixts},
            //      1: {mat: [0: m00_2, ...]},
            //      2: ...
            //    ]
            // }
            onTriggered: {
                console.log("executing");
                checkout.doOperation("transform", {
                    target: currentEntitydataTransform.path,
                    mode: "absolute",
                    tf: {
                        mat:[100,   0,   0,   0,
                               0, 100,   0,   0,
                               0,   0, 100,   0,
                               0,   0,   0,   1]
                    }
                });
            }
        }
        QCtl.Menu {
            id: contextMenu
            QCtl.MenuItem {
                action: transformAction
            }
        }

        QCtl.TreeView {
            id: treeViewCheckout
            model: UPNS.RootTreeModel {
                root: checkout
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
                                contextMenu.popup()
                            }
                        }
                    }
                }
            }
        }
        Rectangle {
            Layout.fillHeight: true
            color: "red"
        }
    }

    SystemPalette {
        id: palette
    }
}
