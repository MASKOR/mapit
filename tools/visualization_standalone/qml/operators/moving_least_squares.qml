import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

import ".."
import "../components"

ColumnLayout {
    id: root
    //// in ////
    property bool editable
    property string currentEntityPath

    function fromParameters(params) {
        entityChooser.currentEntityPath = params.target
        radiusInput.text = params.radius
        polynomialInput.checked = params.polynomial
    }

    //// out ////
    property bool valid: entityChooser.valid && radiusInput.text != ""
    property var parameters: {
        "target": entityChooser.currentEntityPath,
        "radius": parseFloat(radiusInput.text),
        "polynomial": parseInt(polynomialInput.text),
        "computeNormals": computeNormalsInput.checked
    }

    //// UI ////
    HelperTarget {
        id: entityChooser
        currentEntityPath: root.currentEntityPath
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "Radius"
        }
        StyledTextField {
            id: radiusInput
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "0.01"
        }
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "Polynomial Fit"
        }
        StyledTextField {
            id: polynomialInput
            Layout.fillWidth: true
            validator: IntValidator {}
            text: "0.01"
        }
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "Compute Normals"
        }
        StyledCheckBox {
            id: computeNormalsInput
        }
    }
}
