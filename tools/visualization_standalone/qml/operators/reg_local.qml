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
    GridLayout {
        // target
        StyledLabel {
            Layout.column: 0
            Layout.row: 0
            text: "Target:"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 0
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
        StyledTextField {
            Layout.column: 1
            Layout.row: 2
            id: tfPrefix
            Layout.fillWidth: true
            placeholderText: "optional"
            text: ""
        }
        StyledLabel {
            Layout.column: 0
            Layout.row: 3
            text: "frame_id:"
        }

        StyledTextField {
            Layout.column: 1
            Layout.row: 3
            id: frameId
            Layout.fillWidth: true
            placeholderText: "optional"
            text: ""
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
            model:  ["icp", "nothing"]
        }
    }

    // handle result specific config
    GroupBox {
        visible:   handleResultSelect.currentText == "tf-add"
                || handleResultSelect.currentText == "tf-combine"
        title: "Handle result as TF"
        Layout.fillWidth: true
        width: parent.width
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
