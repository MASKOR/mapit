/*******************************************************************************
 *
 * Copyright      2017 Marcus Meeßen	<marcus.meessen@alumni.fh-aachen.de>
 *                2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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
        sourceEntityChooser.currentEntityPath = params.source;
        targetEntityChooser.currentEntityPath = params.target;
        deviationInput.text = params.deviation;
        neighborsInput.text = params.neighbors;
    }

    //// out ////
    property bool valid: sourceEntityChooser.currentEntityPath != ""
                         && targetEntityChooser.currentEntityPath != ""
                         && isFinite(parseFloat(deviationInput.text))
                         && parseInt(neighborsInput.text) > 0
    property var parameters: {
        "source": sourceEntityChooser.currentEntityPath,
        "target": targetEntityChooser.currentEntityPath,
        "deviation": parseFloat(deviationInput.text),
        "neighbors": parseInt(neighborsInput.text)
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
            currentEntityPath: sourceEntityChooser.currentEntityPath + "_statout"
                               + "_d" + deviationInput.text + "_k" + neighborsInput.text
        }
    }

    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "deviation (σ)"
        }
        StyledTextField {
            id: deviationInput
            Layout.fillWidth: true
            validator: DoubleValidator {}
            text: "1.65"
        }
    }

    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "neighbors (for mean)"
        }
        StyledTextField {
            id: neighborsInput
            Layout.fillWidth: true
            validator: IntValidator {}
            text: "20"
        }
    }
}
