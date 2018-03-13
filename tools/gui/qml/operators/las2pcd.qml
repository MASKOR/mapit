/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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
        entityChooser.currentEntityPath = params.target
        demeanCheckbox.checked = params.demean;
        normalizeCheckbox.checked = params.normalize;
    }

    //// out ////
    property bool valid: entityChooser.valid
    property var parameters: {
        "target":entityChooser.currentEntityPath,
        "demean": demeanCheckbox.checked,
        "normalize": normalizeCheckbox.checked,
        "normalizeScale": parseFloat("1.0") //TODO
    }

    //// UI ////
    HelperTarget {
        Layout.fillWidth: true
        id: entityChooser
        currentEntityPath: root.currentEntityPath
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "Demean:"
        }
        CheckBox {
            id: demeanCheckbox
            Layout.fillWidth: true
        }
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            text: "Normalize"
        }
        CheckBox {
            id: normalizeCheckbox
            Layout.fillWidth: true
        }
    }
    SystemPalette {
        id: palette
    }
}
