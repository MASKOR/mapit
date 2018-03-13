/*******************************************************************************
 *
 * Copyright      2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

    //// out ////
    property bool valid:   true
    property var parameters: {
                "target"   : entityChooserTarget.currentEntityPath,
                "tf-storage-prefix" : tfStoragePrefix.currentEntityPath,
                "frame_id" : frameId.currentText,
                "x-min" : parseFloat(xMin.text),
                "x-max" : parseFloat(xMax.text),
                "y-min" : parseFloat(yMin.text),
                "y-max" : parseFloat(yMax.text),
                "z-min" : parseFloat(zMin.text),
                "z-max" : parseFloat(zMax.text),
                "roll-min" : parseFloat(rollMin.text),
                "roll-max" : parseFloat(rollMax.text),
                "pitch-min" : parseFloat(pitchMin.text),
                "pitch-max" : parseFloat(pitchMax.text),
                "yaw-min" : parseFloat(yawMin.text),
                "yaw-max" : parseFloat(yawMax.text)

    }

    //// UI ////
    GridLayout {
        z: 100
        // target
        StyledLabel {
            Layout.column: 0
            Layout.row: 0
            text: "Target"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 0
            Layout.columnSpan: 2
            z: 110
            id: entityChooserTarget
            Layout.fillWidth: true
            currentEntityPath: root.currentEntityPath
        }
        // tf-storage-prefix
        StyledLabel {
            Layout.column: 0
            Layout.row: 1
            text: "tf-storage-prefix"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 1
            Layout.columnSpan: 2
            z: 100
            id: tfStoragePrefix
            Layout.fillWidth: true
            currentEntityPath: "/tf/"
        }
        // frame_id
        StyledLabel {
            Layout.column: 0
            Layout.row: 2
            text: "frame_id:"
        }
        FrameIdChooser {
            Layout.column: 1
            Layout.row: 2
            Layout.columnSpan: 2
            z: 90
            Layout.fillWidth: true
            Layout.minimumWidth: 100
            id: frameId
            allowNew: true
            currentCheckout: root.currentCheckout
            currentText: "random"
        }

        // min/max values
        StyledLabel {
            Layout.column: 1
            Layout.row: 3
            text: "min:"
        }
        StyledLabel {
            Layout.column: 2
            Layout.row: 3
            text: "max:"
        }
        // x
        StyledLabel {
            Layout.column: 0
            Layout.row: 4
            text: "x:"
        }
        StyledTextField {
            Layout.column: 1
            Layout.row: 4
            id: xMin
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "-1.0"
        }
        StyledTextField {
            Layout.column: 2
            Layout.row: 4
            id: xMax
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "1.0"
        }
        // y
        StyledLabel {
            Layout.column: 0
            Layout.row: 5
            text: "y:"
        }
        StyledTextField {
            Layout.column: 1
            Layout.row: 5
            id: yMin
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "-1.0"
        }
        StyledTextField {
            Layout.column: 2
            Layout.row: 5
            id: yMax
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "1.0"
        }
        // z
        StyledLabel {
            Layout.column: 0
            Layout.row: 7
            text: "z:"
        }
        StyledTextField {
            Layout.column: 1
            Layout.row: 7
            id: zMin
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "-1.0"
        }
        StyledTextField {
            Layout.column: 2
            Layout.row: 7
            id: zMax
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "1.0"
        }
        // roll
        StyledLabel {
            Layout.column: 0
            Layout.row: 8
            text: "roll:"
        }
        StyledTextField {
            Layout.column: 1
            Layout.row: 8
            id: rollMin
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "-0.1"
        }
        StyledTextField {
            Layout.column: 2
            Layout.row: 8
            id: rollMax
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "0.1"
        }
        // pitch
        StyledLabel {
            Layout.column: 0
            Layout.row: 9
            text: "pitch:"
        }
        StyledTextField {
            Layout.column: 1
            Layout.row: 9
            id: pitchMin
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "-0.1"
        }
        StyledTextField {
            Layout.column: 2
            Layout.row: 9
            id: pitchMax
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "0.1"
        }
        // yaw
        StyledLabel {
            Layout.column: 0
            Layout.row: 10
            text: "yaw:"
        }
        StyledTextField {
            Layout.column: 1
            Layout.row: 10
            id: yawMin
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "-0.1"
        }
        StyledTextField {
            Layout.column: 2
            Layout.row: 10
            id: yawMax
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "0.1"
        }
    }
}
