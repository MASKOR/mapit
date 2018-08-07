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
    property var parameters

    function beforeOperation() {
        multiInputHelper.beforeOperation()
        parameters = multiInputHelper.parameters;

        parameters["target"] = entityChooserTarget.currentEntityPath;
        parameters["tf-prefix"] = tfPrefix.currentEntityPath;
        parameters["frame_id"] = frameId.currentText;
        parameters["resolution"] = parseFloat(resolution.text);
    }

    HelperMultiInput {
        id: multiInputHelper
        z: 200
        fieldName: "input"
    }

    // algorithm specific, ICP
    GroupBox {
        visible: true
        title: ""
        Layout.fillWidth: true
        width: parent.width
        z: 149
        GridLayout {
            anchors.fill: parent
            // target
            StyledLabel {
                Layout.column: 0
                Layout.row: 0
                text: "target:"
            }
            EntityChooser {
                Layout.column: 1
                Layout.row: 0
                z: 200
                id: entityChooserTarget
                Layout.fillWidth: true
                currentEntityPath: root.currentEntityPath
            }
            StyledLabel {
                Layout.column: 0
                Layout.row: 1
                text: "tf-prefix:"
            }
            EntityChooser {
                Layout.column: 1
                Layout.row: 1
                z: 175
                id: tfPrefix
                Layout.fillWidth: true
                currentEntityPath: "/tf"
            }
            StyledLabel {
                Layout.column: 0
                Layout.row: 2
                text: "frame_id:"
            }
            FrameIdChooser {
                Layout.column: 1
                Layout.row: 2
                z: 150
                Layout.fillWidth: true
                Layout.minimumWidth: 100
                id: frameId
                allowNew: false
                currentWorkspace: root.currentWorkspace
            }
            StyledLabel {
                Layout.column: 0
                Layout.row: 3
                text: "resolution:"
            }
            StyledTextField {
                Layout.column: 1
                Layout.row: 3
                z: 140
                id: resolution
                Layout.fillWidth: true
                validator: DoubleValidator {}
                text: "0.1"
            }
        }
    }
}
