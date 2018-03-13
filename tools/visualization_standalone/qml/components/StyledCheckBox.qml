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

CheckBox {
    property var overrideControlSize: false
    style: CheckBoxStyle {
        indicator:
//        Rectangle {
//            implicitWidth: overrideControlSize ? 1 : appStyle.controlHeight
//            implicitHeight: overrideControlSize ? 1 : appStyle.controlHeight
//            color: "transparent"
            Rectangle {
                anchors.centerIn: parent
                implicitWidth: overrideControlSize ? 1 : appStyle.controlHeightInner
                implicitHeight: overrideControlSize ? 1 : appStyle.controlHeightInner
                color: control.checked
                       ? appStyle.backgroundHighlightColor
                       : control.hovered
                         ? appStyle.backgroundHighlightColor
                         : appStyle.itemBackgroundColor
                border.width: control.activeFocus ? 2 : 1
                border.color: appStyle.buttonBorderColor
                smooth: false
                radius: appStyle.radius
                Rectangle {
                    visible: control.checked
                    color: appStyle.selectionColor
                    border.color: appStyle.selectionBorderColor
                    radius: 1
                    anchors.margins: 3
                    anchors.fill: parent
                }
            }
//        }
    }
}
