import QtQuick 2.4
import QtQuick.Layouts 1.1
import QtQuick.Controls 1.4

import fhac.upns 1.0 as UPNS

import ".."
import "../components"

Item {
    id: root
    property string detailDialogPath
    onDetailDialogPathChanged: privLoadDialog(detailDialogPath)
    property string currentEntityPath: globalApplicationState.currentEntityPath // to prefill operator
    property var prefillHint
    property var currentCheckout: globalApplicationState.currentCheckout
    property var currentOperatorUiItem
    onCurrentOperatorUiItemChanged: {
        controlHolder.height = currentOperatorUiItem.implicitHeight + appStyle.controlMargin * 2
        currentOperatorUiItem.width = controlHolder.width
        controlColumn.update()
    }

    Item {
        id: priv
        property var controls
        Component.onCompleted: {
            if(typeof priv.controls === "undefined") {
                priv.controls = {}
            }
        }
    }

    function finish( controlComponent ) {
        if (controlComponent.status === Component.Ready) {
            //if( typeof root.currentOperator == "undefined" ) return
            root.currentOperatorUiItem = controlComponent.createObject(controlHolder, {currentCheckout: root.currentCheckout,
                                                                                       currentCheckoutName: root.currentCheckout.name,
                                                                                       currentEntityPath: root.currentEntityPath,
                                                                                       shown: true,
                                                                                       prefill: root.prefillHint})
            if (root.currentOperatorUiItem === null) {
                // Error Handling
                console.log("Error creating detailsView")
            }

        } else if (controlComponent.status === Component.Error) {
            // Error Handling
            console.log("Error loading detailsView:", controlComponent.errorString())
        }
        else {
            console.log("Error loading detailsView (Pending?):", controlComponent.errorString())
        }
    }

    Connections {
        target: root.currentOperatorUiItem ? root.currentOperatorUiItem : null
        onImplicitHeightChanged: {
            controlHolder.height = currentOperatorUiItem.implicitHeight + appStyle.controlMargin * 2
            currentOperatorUiItem.width = controlHolder.width
            controlColumn.update()
        }
    }

    function privLoadDialog(pathName) {
        if(typeof priv.controls === "undefined") {
            priv.controls = {}
        }

        var controlComponent = priv.controls[pathName]
        if( typeof controlComponent == "undefined" ) {

            //TODO: this should look in a folder and not only in qrc resources!
            controlComponent = Qt.createComponent(pathName + ".qml")
            //turn caching off temporarily for development
            //priv.controls[renderWidget.selectedGraphicObject.objectType] = controlComponent
        }
        if(typeof root.currentOperatorUiItem !== "undefined" && typeof root.currentOperatorUiItem.shown !== "undefined") {
            root.currentOperatorUiItem.shown = false
        }
        for(var i = controlHolder.children.length; i > 0 ; i--) {
            if(typeof controlHolder.children[i-1].shown !== "undefined") {
                controlHolder.children[i-1].shown = false
            }
        }
        controlHolder.children = "";
//        for(var i = controlHolder.children.length; i > 0 ; i--) {
//            controlHolder.children[i-1].destroy()
//        }
        if (controlComponent.status === Component.Ready) {
            finish(controlComponent);
        } else if (controlComponent.status === Component.Error) {
            console.log("Error: " + controlComponent.errorString() )
        } else {
            controlComponent.statusChanged.connect(function() {finish(controlComponent)})
        }
    }

    Item {
        enabled: !globalRepository.isLoadingOperators
        id: controlColumn
        anchors.left: parent.left
        anchors.right: parent.right
        onWidthChanged: controlHolder.width = width
        z: 100
//        StyledLabel {
//            id: topLabel
//            anchors.top: parent.top
//            anchors.left: parent.left
//            anchors.right: parent.right
//            anchors.margins: appStyle.controlMargin
//            text: currentOperator ? currentOperator.moduleName : pipelineName
//        }
        Item {
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.margins: appStyle.controlMargin
            id: controlHolder
            z: 100
            onWidthChanged: { if(currentOperatorUiItem) currentOperatorUiItem.width = width }
        }
        RowLayout {
            anchors.top: controlHolder.bottom
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.margins: appStyle.controlMargin
            visible: !globalApplicationState.currentDetailDialogHasExecuteButton
            StyledButton {
                Layout.leftMargin: appStyle.controlMargin
                text: "Execute"
                enabled: currentCheckout ? currentOperatorUiItem ? currentOperatorUiItem.valid && !currentCheckout.isBusyExecuting : false : false
                onClicked: {
                    var dialogParts = globalApplicationState.currentDetailDialog.split("/")
                    var moduleName = dialogParts[dialogParts.length-1]
                    console.log("executing " + moduleName)
                    currentCheckout.doOperation(moduleName, currentOperatorUiItem.parameters)
                }
                BusyIndicator {
                    anchors.centerIn: parent
                    running: currentCheckout.isBusyExecuting
                }
            }
        }
        Item {
            Layout.fillHeight: true
        }
    }
}
