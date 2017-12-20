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
    property bool valid:   entityChooserTarget.valid
                        && entityChooserInput.valid
    property var parameters: {
        "target":    entityChooserTarget.currentEntityPath,
        "input":     "[" + entityChooserTarget.currentEntityPath + "]",
        "tf-prefix": tfPrefix.currentEntityPath,
    }

    //// UI ////
    HelperTarget {
        z: 101
        Layout.fillWidth: true
        id: entityChooserTarget
        currentEntityPath: root.currentEntityPath
    }
    // input
    HelperTarget {
        nameOverwrite: "Input:"
        Layout.fillWidth: true
        id: entityChooserInput
        currentEntityPath: root.currentEntityPath
    }
    // tf setup and general cfg
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "tf-prefix:"
        }
        StyledTextField {
            id: tfPrefix
            Layout.fillWidth: true
            placeholderText: "optional"
            text: ""
        }
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "frame_id:"
        }

        StyledTextField {
            id: frameId
            Layout.fillWidth: true
            placeholderText: "optional"
            text: ""
        }
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "use-metascan:"
        }
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
    // choose algorithm
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "handle-result:"
        }
        StyledComboBox {
            id: handleResultSelect
            model:  ["tf-add", "tf-combine", "data-change"]
        }
    }
    ColumnLayout {
        Layout.fillWidth: true
        visible:   handleResultSelect.currentText == "tf-add"
                || handleResultSelect.currentText == "tf-combine"
        RowLayout {
            StyledLabel {
                text: "tf-is_static:"
            }
            StyledCheckBox {
                id: tfISstatic
                onCheckedChanged: if(checked) kCb.checked = false
            }
        }
        RowLayout {
            StyledLabel {
                text: "tf-frame_id:"
            }
            FrameIdChooser {
                Layout.fillWidth: true
                Layout.minimumWidth: 100
                id: tfFrameId
                allowNew: false
                currentCheckout: root.currentCheckout
            }
        }
        RowLayout {
            StyledLabel {
                text: "tf-child_frame_id:"
            }
            FrameIdChooser {
                Layout.fillWidth: true
                Layout.minimumWidth: 100
                id: tfChildFrameId
                allowNew: false
                currentCheckout: root.currentCheckout
            }
        }
    }

    // choose matching algorithm
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "matching-algorithm:"
        }
        StyledComboBox {
            id: matchingAlgorithm
            model:  ["icp", "nothing"]
        }
    }
    // algorithm specific, ICP
    GridLayout {
        Layout.fillWidth: true
        visible:   matchingAlgorithm.currentText == "icp"
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
