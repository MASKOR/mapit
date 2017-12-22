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
        entityChooserTarget.currentEntityPath = params.target
        entityChooserInput.currentEntityPath = params.target
    }

    //// out ////
    property bool valid:   true
    property var parameters: {
        "target":    entityChooserTarget.currentEntityPath,
        "input":     entityChooserInput.currentEntityPath,
        "tf-prefix": tfPrefix.currentEntityPath,
        "frame_id":  frameId.currentText,

        "use-metascan": useMetascan.checked,
        "voxelgrid": useMetascanVoxelgrid.text,

        "handle-result": handleResultSelect.currentText,

        "tf-frame_id": tfFrameId.currentText,
        "tf-child_frame_id": tfChildFrameId.currentText,
        "tf-is_static": tfISstatic.checked,

        "matching-algorithm": matchingAlgorithm.currentText,

        "icp-maximum-iterations": parseInt(icpMaximumIterations.text),
        "icp-max-correspondence-distance": parseFloat(icpMaxCorrespondenceDistance.text),
        "icp-transformation-epsilon": parseFloat(icpTransformationEpsilon.text)
    }

    //// UI ////
    GridLayout {
        z: 189
        // target
        StyledLabel {
            Layout.column: 0
            Layout.row: 0
            text: "Target:"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 0
            z: 189
            id: entityChooserTarget
            Layout.fillWidth: true
            currentEntityPath: root.currentEntityPath
        }
        // input
        StyledLabel {
            Layout.column: 0
            Layout.row: 1
            text: "Input:"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 1
            z: 188
            id: entityChooserInput
            Layout.fillWidth: true
            currentEntityPath: root.currentEntityPath
        }
        // tf setup and general cfg
        StyledLabel {
            Layout.column: 0
            Layout.row: 2
            text: "tf-prefix:"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 2
            z: 169
            id: tfPrefix
            Layout.fillWidth: true
            currentEntityPath: ""
        }
        StyledLabel {
            Layout.column: 0
            Layout.row: 3
            text: "frame_id:"
        }
        FrameIdChooser {
            Layout.column: 1
            Layout.row: 3
            z: 168
            Layout.fillWidth: true
            Layout.minimumWidth: 100
            id: frameId
            allowNew: false
            currentCheckout: root.currentCheckout
        }
        StyledLabel {
            Layout.column: 0
            Layout.row: 4
            text: "use-metascan:"
        }
        RowLayout {
            Layout.column: 1
            Layout.row: 4
            StyledCheckBox {
                id: useMetascan
                onCheckedChanged: if(checked) kCb.checked = false
            }
            StyledLabel {
                visible: useMetascan.checked
                text: "voxelgrid:"
            }
            StyledTextField {
                visible: useMetascan.checked
                id: useMetascanVoxelgrid
                Layout.fillWidth: true
                validator: DoubleValidator {}
                placeholderText: "not implemented"
    //            text: "0.3"
            }
        }
        // choose how to handle the result
        StyledLabel {
            Layout.column: 0
            Layout.row: 5
            text: "handle-result:"
        }
        StyledComboBox {
            Layout.column: 1
            Layout.row: 5
            Layout.fillWidth: true
            id: handleResultSelect
            model:  ["tf-add", "tf-combine", "data-change"]
        }
        // choose matching algorithm
        StyledLabel {
            Layout.column: 0
            Layout.row: 6
            text: "matching-algorithm:"
        }
        StyledComboBox {
            Layout.column: 1
            Layout.row: 6
            Layout.fillWidth: true
            id: matchingAlgorithm
            model:  ["icp", "nothing (will not work, just for GUI)"]
        }
    }

    // handle result specific config
    GroupBox {
        visible:   handleResultSelect.currentText == "tf-add"
                || handleResultSelect.currentText == "tf-combine"
        title: "Handle result as TF"
        Layout.fillWidth: true
        width: parent.width
        z: 159
        GridLayout {
            anchors.fill: parent
            StyledLabel {
                Layout.column: 0
                Layout.row: 0
                text: "tf-is_static:"
            }
            StyledCheckBox {
                Layout.column: 1
                Layout.row: 0
                z: 159
                Layout.fillWidth: true
                id: tfISstatic
                onCheckedChanged: if(checked) kCb.checked = false
            }
            StyledLabel {
                Layout.column: 0
                Layout.row: 1
                text: "tf-frame_id:"
            }
            FrameIdChooser {
                Layout.column: 1
                Layout.row: 1
                z: 158
                Layout.fillWidth: true
                Layout.minimumWidth: 100
                id: tfFrameId
                allowNew: false
                currentCheckout: root.currentCheckout
            }
            StyledLabel {
                Layout.column: 0
                Layout.row: 2
                text: "tf-child_frame_id:"
            }
            FrameIdChooser {
                Layout.column: 1
                Layout.row: 2
                z: 157
                Layout.fillWidth: true
                Layout.minimumWidth: 100
                id: tfChildFrameId
                allowNew: false
                currentCheckout: root.currentCheckout
            }
        }
    }

    // algorithm specific, ICP
    GroupBox {
        visible:   matchingAlgorithm.currentText == "icp"
        title: "ICP configs"
        Layout.fillWidth: true
        width: parent.width
        z: 149
        GridLayout {
            anchors.fill: parent
            StyledLabel {
                Layout.column: 0
                Layout.row: 0
                text: "maximum-iterations:"
            }
            StyledTextField {
                Layout.column: 1
                Layout.row: 0
                id: icpMaximumIterations
                Layout.fillWidth: true
                validator: IntValidator {}
                placeholderText: "optional"
                text: ""
            }
            StyledLabel {
                Layout.column: 0
                Layout.row: 1
                text: "max-correspondence-distance:"
            }
            StyledTextField {
                Layout.column: 1
                Layout.row: 1
                id: icpMaxCorrespondenceDistance
                Layout.fillWidth: true
                validator: DoubleValidator {}
                placeholderText: "optional"
                text: ""
            }
            StyledLabel {
                Layout.column: 0
                Layout.row: 2
                text: "max-transformation-epsilon:"
            }
            StyledTextField {
                Layout.column: 1
                Layout.row: 2
                id: icpTransformationEpsilon
                Layout.fillWidth: true
                validator: DoubleValidator {}
                placeholderText: "optional"
                text: ""
            }
        }
    }
}
