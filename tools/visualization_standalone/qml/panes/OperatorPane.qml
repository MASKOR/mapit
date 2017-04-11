import QtQuick 2.4
import QtQuick.Layouts 1.1
import QtQuick.Controls 1.4

Item {
    id: root
    property var currentOperator
    property var currentCheckout
    property var currentOperatorUiItem
    Item {
        id:priv
        property var controls
        Component.onCompleted: {
            controls = {};
        }
    }

    function finish( controlComponent ) {
        if (controlComponent.status === Component.Ready) {
            if( typeof root.currentOperator == "undefined" ) return;
            root.currentOperatorUiItem = controlComponent.createObject(controlHolder, {currentOperator: root.currentOperator});
            if (root.currentOperatorUiItem === null) {
                // Error Handling
                console.log("Error creating detailsView");
            }
            root.currentOperatorUiItem.width = controlHolder.width;
            root.currentOperatorUiItem.height = controlHolder.height;

        } else if (controlComponent.status === Component.Error) {
            // Error Handling
            console.log("Error loading detailsView:", controlComponent.errorString());
        }
        else {
            console.log("Error loading detailsView (Pending?):", controlComponent.errorString());
        }
    }

    onCurrentOperatorChanged: {
        if( root.currentOperator === null || typeof root.currentOperator == "undefined") {
            controlHolder.children = "";
            return;
        }

        var controlComponent = priv.controls[currentOperator.moduleName];
        if( typeof controlComponent == "undefined" ) {

            //TODO: this should look in a folder and not only in qrc resources!
            controlComponent = Qt.createComponent("../operators/" + currentOperator.moduleName + ".qml");
            //turn caching off temporarily for development
            //priv.controls[renderWidget.selectedGraphicObject.objectType] = controlComponent;
        }
        controlHolder.children = "";
//        for(var i = controlHolder.children.length; i > 0 ; i--) {
//            controlHolder.children[i-1].destroy();
//        }
        if (controlComponent.status === Component.Ready) {
            finish(controlComponent);
        } else if (controlComponent.status === Component.Error) {
            console.log("Error: " + controlComponent.errorString() );
        } else {
            controlComponent.statusChanged.connect(function() {finish(controlComponent);});
        }
    }
    RowLayout {
        anchors.fill: parent
        Item {
            Layout.fillHeight: true
            Layout.fillWidth: true
            id: controlHolder
        }
        ColumnLayout {
            Layout.fillHeight: true
            Button {
                text: "Execute"
                enabled: currentOperatorUiItem.valid
                onClicked: {
                    currentCheckout.doOperation(currentOperator.moduleName, currentOperatorUiItem.parameter);
                }
            }
        }
    }
}
