import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1

import fhac.upns 1.0
import "../components"

Button {
    id: root

    default property alias controls: controlsLayout.children

    signal initializeDialog
    signal operationFinished(var result)
    signal buildOperationDescription
    property var operationDescription
    property var operationResult

    //text: "load_pointcloud"
    onClicked: {
        settingsDialog.open();
    }
    Dialog {
        id: settingsDialog
        height: 30
        standardButtons: StandardButton.Ok | StandardButton.Cancel
        onVisibleChanged: {
            if(visible) {
                root.initializeDialog()
            }
        }
        onAccepted: {
            root.buildOperationDescription()
            var opdesc = root.operationDescription
            var result = Globals.doOperation( opdesc, function(result) {
                if(result.status !== 0) {
                    var errMsg = "An operation returned an error.\n";
                    errMsg += "See the log file for more information.\n";
                    errMsg += "\n"
                    errMsg += "(Code: "+result.status+")"
                    errorDialog.showError(errMsg);
                    return
                }
                root.operationResult = result
                root.operationFinished(result)
            });
        }
        ColumnLayout {
            id: controlsLayout
            //anchors.fill: parent // dialog adapts it's size
        }
    }
    Dialog {
        id: errorDialog
        property alias text: errorText.text
        title: "Error"
        standardButtons: StandardButton.Ok
        visible: false
        Text {
            id: errorText
            renderType: Text.NativeRendering
            color: palette.text
        }
        function showError(msg) {
            text = msg
            visible = true
        }
    }
    SystemPalette {
        id: palette
    }
}
