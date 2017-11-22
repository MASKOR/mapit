import QtQuick 2.4
import QtQuick.Layouts 1.1
import QtQuick.Controls 1.4

import fhac.upns 1.0 as UPNS

import ".."

// Can load two types of dialogs:
// Operator: the dialog must fill parameters, "Execute" Button is provided by this Pane.
// Pipeline: Dialog executes on its own
Item {
    id: root
    property var currentEntityPath // to prefill operator
    property var prefillHint
    property string pipelineName
    property var currentOperator
    property var currentCheckout
    property var currentOperatorUiItem
    onCurrentOperatorUiItemChanged: {
        controlHolder.height = currentOperatorUiItem.implicitHeight + appStyle.controlMargin * 2
        currentOperatorUiItem.width = controlHolder.width
        controlColumn.update()
    }

    function loadOperatorDialog(operator, prefillHintParameters) {
        prefillHint = prefillHintParameters
        pipelineName = ""
        currentOperator = operator
    }
    function loadPipelineDialog(pipelineNam, prefillHintParameters) {
        prefillHint = prefillHintParameters
        currentOperator = null
        pipelineName = pipelineNam
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
            root.currentOperatorUiItem = controlComponent.createObject(controlHolder, {currentOperator: root.currentOperator,
                                                                                       pipelineName: root.pipelineName,
                                                                                       currentCheckout: root.currentCheckout,
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
        target: root.currentOperatorUiItem
        onImplicitHeightChanged: {
            controlHolder.height = currentOperatorUiItem.implicitHeight + appStyle.controlMargin * 2
            currentOperatorUiItem.width = controlHolder.width
            controlColumn.update()
        }
    }

    function loadDialog(path, name) {
        if(typeof priv.controls === "undefined") {
            priv.controls = {}
        }

        var controlComponent = priv.controls[name]
        if( typeof controlComponent == "undefined" ) {

            //TODO: this should look in a folder and not only in qrc resources!
            controlComponent = Qt.createComponent(path + name + ".qml")
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

    onPipelineNameChanged: {
        if( root.pipelineName === null || root.pipelineName === "" || typeof root.pipelineName == "undefined") {
            controlHolder.children = ""
            return;
        }
        currentOperator = null
        loadDialog("../pipelines/", root.pipelineName)
    }

    onCurrentOperatorChanged: {
        if( root.currentOperator === null || typeof root.currentOperator == "undefined") {
            controlHolder.children = ""
            return;
        }
        pipelineName = ""
        loadDialog("../operators/", currentOperator.moduleName)
    }
    Item {
        enabled: !globalRepository.isLoadingOperators
        id: controlColumn
        anchors.left: parent.left
        anchors.right: parent.right
        onWidthChanged: controlHolder.width = width
        z: 100
        StyledLabel {
            id: topLabel
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.margins: appStyle.controlMargin
            text: currentOperator ? currentOperator.moduleName : pipelineName
        }
        Item {
            anchors.top: topLabel.bottom
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.margins: appStyle.controlMargin
            id: controlHolder
            z: 100
            onWidthChanged: { currentOperatorUiItem.width = width }
        }
        RowLayout {
            anchors.top: controlHolder.bottom
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.margins: appStyle.controlMargin
            visible: root.currentOperator !== null && typeof root.currentOperator !== "undefined"
            StyledButton {
                Layout.leftMargin: appStyle.controlMargin
                text: "Execute"
                enabled: currentCheckout?currentOperatorUiItem?currentOperatorUiItem.valid : false : false
                onClicked: {
                    currentCheckout.doOperation(currentOperator.moduleName, currentOperatorUiItem.parameters)
                }
            }
        }
        Item {
            Layout.fillHeight: true
        }
    }
}
