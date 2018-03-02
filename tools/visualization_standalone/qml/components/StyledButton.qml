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

import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4

Button {
    id: root
    property bool isIcon: false
    property string iconSource
    property color color: appStyle.itemBackgroundColor
    property bool useHoverHighlight: true
    implicitHeight: appStyle.controlHeightOuter
    style: ButtonStyle {
        label: Label {
            visible: !control.isIcon
            renderType: Text.NativeRendering
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            text: control.text
            color: enabled ? appStyle.textColor : appStyle.textColorDisabled
            font.family: appStyle.labelFontFamily
            font.weight: appStyle.labelFontWeight
            font.pixelSize: appStyle.labelFontPixelSize
        }

        background: Rectangle {
            implicitWidth: control.isIcon ? appStyle.controlHeightOuter : appStyle.controlHeightOuter*4.0
            implicitHeight: appStyle.controlHeightOuter
            color: control.checked
                   ? appStyle.backgroundHighlightColor
                   : control.hovered && control.useHoverHighlight
                     ? appStyle.backgroundHighlightColor
                     : control.isIcon
                       ? "transparent"
                       : control.color
            border.width: control.activeFocus ? 2 : 1
            border.color: control.isIcon ? "transparent" : appStyle.buttonBorderColor
            smooth: false
            radius: appStyle.radius
            Image {
                id: img
                mipmap: false
                smooth: false
                sourceSize: Qt.size(appStyle.iconSize, appStyle.iconSize)
                width: appStyle.iconSize
                height: appStyle.iconSize
                x: 0
                y: 0
                anchors.centerIn: parent
                visible: control.isIcon
                opacity: control.enabled ? 1.0 : 0.5
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                source: control.iconSource && control.iconSource !== "" ? control.iconSource : "image://icon/"
            }
        }
    }
}
