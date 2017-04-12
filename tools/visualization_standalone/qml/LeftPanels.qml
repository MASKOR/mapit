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
        name: checkoutChooser.currentCheckoutName?checkoutChooser.currentCheckoutName : "testcheckout"
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

        CheckoutChooser {
            id: checkoutChooser
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

        CheckoutTreeView {
            id: treeViewCheckout
            currentCheckout: checkout
            contextMenu: QCtl.Menu {
                id: contextMenu
                QCtl.MenuItem {
                    action: transformAction
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
