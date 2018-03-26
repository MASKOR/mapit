/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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

RowLayout {
    property var popupLayer: appStyle.popupLayerRight
    property alias text: label.text
    // second part ensures, that no top level entities can be created. This ensures compatibility to tfs.
    property bool valid: entityChooser.currentEntityPath != "" && entityChooser.currentEntityPath.substring(1).indexOf('/') != -1
    property alias currentEntityPath: entityChooser.currentEntityPath
    property string nameOverwrite: ""
    Layout.fillWidth: true
    //z: 100
    StyledLabel {
        id: label
        text: nameOverwrite === "" ? "Target:" : nameOverwrite
    }
    EntityChooser {
        id: entityChooser
        Layout.fillWidth: true
        popupLayer: parent.popupLayer
    }
}
