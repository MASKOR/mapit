/*******************************************************************************
 *
 * Copyright 2017-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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

import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls.Private 1.0

// This file contains private Qt Quick modules that might change in future versions of Qt
// Tested on: Qt 5.10

Label {
    //Note: If tooltips are enabled, the label can not be clicked through
    property alias tooltip: tooltipMousearea.text
    color: enabled ? appStyle.textColor : appStyle.textColorDisabled
    font.family: appStyle.labelFontFamily
    font.weight: appStyle.labelFontWeight
    font.pixelSize: appStyle.labelFontPixelSize
    height: appStyle.controlHeightOuter
    verticalAlignment: Label.AlignVCenter
    MouseArea {
        id: tooltipMousearea
        property string text

        anchors.fill: parent
        hoverEnabled: tooltipMousearea.enabled

        acceptedButtons: Qt.NoButton
        onExited: Tooltip.hideText()
        onCanceled: Tooltip.hideText()

        Timer {
            interval: 200
            running: tooltipMousearea.enabled && tooltipMousearea.containsMouse && tooltipMousearea.text.length
            onTriggered: Tooltip.showText(tooltipMousearea, Qt.point(tooltipMousearea.mouseX, tooltipMousearea.mouseY), tooltipMousearea.text)
        }
    }
}
