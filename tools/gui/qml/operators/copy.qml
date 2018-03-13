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

    function fromParameters(params) {
        entityChooserSource.currentEntityPath = params.target
    }

    //// out ////
    property bool valid:   true
    property var parameters: {
        "source":    entityChooserSource.currentEntityPath,
        "target":    entityChooserTarget.currentEntityPath
    }

    //// UI ////
    GridLayout {
        z: 100
        // source
        StyledLabel {
            Layout.column: 0
            Layout.row: 1
            text: "Source:"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 1
            z: 100
            id: entityChooserSource
            Layout.fillWidth: true
            currentEntityPath: root.currentEntityPath
        }
        // target
        StyledLabel {
            Layout.column: 0
            Layout.row: 2
            text: "Target:"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 2
            z: 99
            id: entityChooserTarget
            Layout.fillWidth: true
        }
    }
}
