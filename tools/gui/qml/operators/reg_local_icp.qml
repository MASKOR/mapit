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
        registrationHelper.beforeOperation()
        parameters = registrationHelper.parameters;

        parameters["icp-maximum-iterations"] = parseInt(icpMaximumIterations.text);
        parameters["icp-max-correspondence-distance"] = parseFloat(icpMaxCorrespondenceDistance.text);
        parameters["icp-transformation-epsilon"] = parseFloat(icpTransformationEpsilon.text);
        parameters["icp-euclidean-fitness-epsilon"] = parseFloat(icpEuclideanFitnessEpsilon.text);
    }

    HelperRegistration {
        id: registrationHelper
        z: 200
        local: true
    }

    // algorithm specific, ICP
    GroupBox {
        visible: true
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
