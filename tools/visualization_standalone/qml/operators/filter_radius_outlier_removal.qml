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
        sourceEntityChooser.currentEntityPath = params.source;
        targetEntityChooser.currentEntityPath = params.target;
        radiusInput.text = params.radius;
        minimumNeighborsInput.text = params.minimumNeighbors;
    }

    //// out ////
    property bool valid: sourceEntityChooser.currentEntityPath != ""
                         && targetEntityChooser.currentEntityPath != ""
                         && parseFloat(radiusInput.text) > 0.0
                         && parseInt(minimumNeighborsInput.text) > 0
    property var parameters: {
        "source": sourceEntityChooser.currentEntityPath,
        "target": targetEntityChooser.currentEntityPath,
        "radius": parseFloat(radiusInput.text),
        "minimumNeighbors": parseInt(minimumNeighborsInput.text)
    }

    //// UI ////
    RowLayout {
        Layout.fillWidth: true
        z: 20
        StyledLabel {
            text: "Source:"
        }
        EntityChooser {
            id: sourceEntityChooser
            Layout.fillWidth: true
            currentEntityPath: root.currentEntityPath
        }
    }

    RowLayout {
        Layout.fillWidth: true
        z: 10
        StyledLabel {
            text: "Target:"
        }
        EntityChooser {
            id: targetEntityChooser
            Layout.fillWidth: true
            currentEntityPath: sourceEntityChooser.currentEntityPath + "_radout"
                               + "_r" + radiusInput.text + "_k" + minimumNeighborsInput.text
        }
    }

    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "radius"
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
            text: "minimum neighbors"
        }
        StyledTextField {
            id: minimumNeighborsInput
            Layout.fillWidth: true
            validator: IntValidator {}
            text: "20"
        }
    }
}
