import QtQuick 2.4
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtQuick.Window 2.2 as Wnd
import QtGraphicalEffects 1.0
import pcl 1.0
import fhac.upns 1.0 as UPNS
import QtQuick 2.0 as QQ2
import fhac.upns 1.0 as UPNS
import "panes"
import "components"

Item {
    id: root
//    property alias currentOperator: operatorListPane.currentOperator
//    property alias currentPipeline: operatorListPane.currentPipeline
//    property alias visibleElems: treeViewCheckout.visibleElems
//    property string currentEntityPath: treeViewCheckout.currentIndex && treeViewCheckout.model.data(treeViewCheckout.currentIndex, UPNS.RootTreeModel.NodeTypeRole) === UPNS.RootTreeModel.EntityNode ? treeViewCheckout.model.data(treeViewCheckout.currentIndex, Qt.ToolTipRole) : ""
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
//                QCtl.Action {
//                    //TODO
//                    id: transformAction
//                    text: "&Transform"
//                    shortcut: "T"
//                    //iconName: "edit-copy"
//                    enabled: currentEntitydataTransform.path.length > 0
//                    //    target: "path/to/target.tf"
//                    //    mode: "relative"|"absolute",
//                    //    tf: [
//                    //      0: {mat: [0: m00, 1: m01, m02...], parent: "/path/to/parent", timestamp: unixts},
//                    //      1: {mat: [0: m00_2, ...]},
//                    //      2: ...
//                    //    ]
//                    // }
//                    onTriggered: {
//                        console.log("executing");
//                        checkout.doOperation("transform", {
//                            target: currentEntitydataTransform.path,
//                            mode: "absolute",
//                            tf: {
//                                mat:[100,   0,   0,   0,
//                                       0, 100,   0,   0,
//                                       0,   0, 100,   0,
//                                       0,   0,   0,   1]
//                            }
//                        });
//                    }
//                }
            }
            ColumnLayout {
                Layout.preferredHeight: 300
                Layout.fillWidth: true
                spacing: appStyle.controlMargin
                StyledHeader {
                    Layout.fillWidth: true
                    id: headerCheckout
                    text: qsTr("Checkout")
                }
                ColumnLayout {
                    Layout.fillWidth: true
                    Layout.leftMargin: appStyle.controlMargin
                    Layout.rightMargin: appStyle.controlMargin
                    spacing: appStyle.controlMargin
                    visible: headerCheckout.checked
                    RowLayout {
                        Layout.fillWidth: true
                        StyledLabel {
                            text: globalApplicationState.currentCheckout.name
                            Layout.fillWidth: true
                        }
                        CheckoutChooser {
                            width: implicitWidth
                            id: checkoutChooser
                            onCurrentCheckoutNameChanged: globalApplicationState.currentCheckoutName = currentCheckoutName
                        }
                    }
                    StyledLabel {
                        visible: !globalRepository.isLoaded
                        text: "No Repository loaded"
                    }
                    CheckoutTreeView {
                        visible: globalRepository.isLoaded
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.bottomMargin: appStyle.controlMargin
                        id: treeViewCheckout
                        onCurrentEntityPathChanged: globalApplicationState.currentEntityPath = currentEntityPath
                        contextMenu: QCtl.Menu {
                            id: contextMenu
//                            QCtl.MenuItem {
//                                action: transformAction
//                            }
                        }
                    }
                }
            }
        }
    }
}
