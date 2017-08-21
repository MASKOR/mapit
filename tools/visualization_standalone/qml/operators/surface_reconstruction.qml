import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

import ".."

Item {
    id: root
    //// in ////
    property bool editable
    property var currentCheckout
    property string currentEntityPath

    function fromParameters(params) {
        entityChooser.currentEntityPath = params.target
        densityInput.text = params.density
        noiseInput.text = params.noise
    }

    //// out ////
    property bool valid: entityChooser.valid && densityInput.acceptableInput && noiseInput.acceptableInput
    property var parameters: {
        "target": entityChooser.currentEntityPath,
        "density": parseFloat(densityInput.text),
        "noise": parseFloat(noiseInput.text)
    }

    //// UI ////
    ColumnLayout {
        anchors.fill: parent
        height: root.height
        HelperTarget {
            id: entityChooser
            currentEntityPath: root.currentEntityPath
            dialogRoot: root
        }
        RowLayout {
            Layout.fillWidth: true
            StyledLabel {
                text: "Density"
            }
            StyledTextField {
                id: densityInput
                Layout.fillWidth: true
                validator: DoubleValidator {}
                text: "0.01"
            }
        }
        RowLayout {
            Layout.fillWidth: true
            StyledLabel {
                text: "Expected Noise"
            }
            StyledTextField {
                id: noiseInput
                Layout.fillWidth: true
                validator: DoubleValidator {}
                text: "0.01"
            }
        }
        Item {
            Layout.fillHeight: true
        }
    }
}
