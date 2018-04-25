/*******************************************************************************
 *
 * Copyright 2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
 *                2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

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
    }

    //// out ////
    property bool valid:   true
    property var parameters: {
        "target":    entityChooserTarget.currentEntityPath,
        "tf-prefix": tfPrefix.currentEntityPath,
        "frame_id":  frameId.currentText,

        "use-metascan": useMetascan.checked,
        "voxelgrid": useMetascanVoxelgrid.text,

        "handle-result": handleResultSelect.currentText,

        "tf-frame_id": tfFrameId.currentText,
        "tf-child_frame_id": tfChildFrameId.currentText,
        "tf-is_static": tfIsStatic.checked,

        "matching-algorithm": matchingAlgorithm.currentText,

        "icp-maximum-iterations": parseInt(icpMaximumIterations.text),
        "icp-max-correspondence-distance": parseFloat(icpMaxCorrespondenceDistance.text),
        "icp-transformation-epsilon": parseFloat(icpTransformationEpsilon.text),
        "icp-euclidean-fitness-epsilon": parseFloat(icpEuclideanFitnessEpsilon.text),
    }

    function beforeOperation() {
        var input = []
        for (var i = 0; i < inputRepeater.model.count; ++i) {
            if (inputRepeater.model.get(i).name) {
                input.push( inputRepeater.model.get(i).name )
            }
        }
        if (entityChooserInput.currentEntityPath !== "") {
            input.push( entityChooserInput.currentEntityPath )
        }

        parameters["input"] = input;
    }

    //// UI ////
    GridLayout {
        z: 189
        // target
        StyledLabel {
            Layout.column: 0
            Layout.row: 1
            text: "Target:"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 1
            z: 200
            id: entityChooserTarget
            Layout.fillWidth: true
            currentEntityPath: root.currentEntityPath
        }
        // input
        Repeater {
            id: inputLabelRepeater
            model: 0
            StyledLabel {
                Layout.column: 0
                Layout.row: 2 + (2 * index)
                text: "Input[" + index + "]"
            }
        }
        Repeater {
            z: 199
            id: inputRepeater
            model: ListModel {}
            EntityChooser {
                Layout.column: 1
                Layout.row: 2 + (2 * index)
                z: 188 - index
                id: entityChooserInputRepeated
                Layout.fillWidth: true
                currentEntityPath: inputRepeater.model.get(index).name
                onCurrentEntityPathChanged: {
                    inputRepeater.model.get(index).name = currentEntityPath
                }
            }
        }

        StyledLabel {
            Layout.column: 0
            Layout.row: 3 + (2 * inputRepeater.count)
            text: "Input[" + inputRepeater.count + "]"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 3 + (2 * inputRepeater.count)
            z: 188 - index
            id: entityChooserInput
            Layout.fillWidth: true
            currentEntityPath: root.currentEntityPath
        }
        Connections {
            target: entityChooserInput.internalTextField
            onEditingFinished: {
                if (entityChooserInput.currentEntityPath) {
                    var entityLastName = entityChooserInput.currentEntityPath
                    entityChooserInput.currentEntityPath = ""

                    inputRepeater.model.append( { "name": entityLastName } )
                    inputLabelRepeater.model = inputRepeater.count
                }
            }
        }
        // tf setup and general cfg
        StyledLabel {
            Layout.column: 0
            Layout.row: 5 + (2 * inputRepeater.count)
            text: "tf-prefix:"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 5 + (2 * inputRepeater.count)
            z: 169
            id: tfPrefix
            Layout.fillWidth: true
            currentEntityPath: ""
        }
        StyledLabel {
            Layout.column: 0
            Layout.row: 7 + (2 * inputRepeater.count)
            text: "frame_id:"
        }
        FrameIdChooser {
            Layout.column: 1
            Layout.row: 7 + (2 * inputRepeater.count)
            z: 168
            Layout.fillWidth: true
            Layout.minimumWidth: 100
            id: frameId
            allowNew: false
            currentWorkspace: root.currentWorkspace
        }
        StyledLabel {
            Layout.column: 0
            Layout.row: 9 + (2 * inputRepeater.count)
            text: "use-metascan:"
        }
        RowLayout {
            Layout.column: 1
            Layout.row: 9 + (2 * inputRepeater.count)
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
            Layout.row: 11 + (2 * inputRepeater.count)
            text: "handle-result:"
        }
        StyledComboBox {
            Layout.column: 1
            Layout.row: 11 + (2 * inputRepeater.count)
            Layout.fillWidth: true
            id: handleResultSelect
            model:  ["tf-add", "tf-combine", "data-change"]
        }
        // choose matching algorithm
        StyledLabel {
            Layout.column: 0
            Layout.row: 13 + (2 * inputRepeater.count)
            text: "matching-algorithm:"
        }
        StyledComboBox {
            Layout.column: 1
            Layout.row: 13 + (2 * inputRepeater.count)
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
                id: tfIsStatic
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
                currentWorkspace: root.currentWorkspace
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
                currentWorkspace: root.currentWorkspace
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
            StyledLabel {
                Layout.column: 0
                Layout.row: 3
                text: "euclidean-fitness-epsilon:"
            }
            StyledTextField {
                Layout.column: 1
                Layout.row: 3
                id: icpEuclideanFitnessEpsilon
                Layout.fillWidth: true
                validator: DoubleValidator {}
                placeholderText: "optional"
                text: ""
            }
        }
    }
}
