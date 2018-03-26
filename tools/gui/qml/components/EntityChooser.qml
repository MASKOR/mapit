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

import QtQuick 2.0
import QtQuick.Controls 1.1
import fhac.mapit 1.0
import QtQuick.Controls.Styles 1.4

import "."

Item {
    id:root
    height: appStyle.controlHeightOuter
    //z: 1000
    property var popupLayer: appStyle.popupLayerRight
    property var currentWorkspace: globalApplicationState.currentWorkspace
    property alias currentEntityPath: filter.currentText
    property real extendedHeight: 200
    property bool allowNewMap: true
    property var internalTextField: filter.internalTextField
    QuickAccessMenu {
        id: filter
        anchors.fill: parent
        allowNew: true
        model: currentWorkspace.entities
        popupLayer: parent.popupLayer
    }
}
