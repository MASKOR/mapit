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

TextField {
    font.pixelSize: appStyle.labelFontPixelSize
    verticalAlignment: TextInput.AlignBottom
    implicitHeight: appStyle.controlHeightOuter
    height: appStyle.controlHeightOuter
    style: TextFieldStyle {
        id: textFieldStyle
        background: //Rectangle {
//            color: "transparent"
//            implicitHeight: appStyle.controlHeightOuter + appStyle.labelFontPixelSize*0.37 // hack for wrong vertical text alignment
//            implicitWidth: appStyle.controlHeightOuter*4
            Rectangle {
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right
                radius: appStyle.radius
                implicitHeight: appStyle.controlHeightOuter
                height: appStyle.controlHeightOuter
                border.color: appStyle.buttonBorderColor
                border.width: 1
                color: appStyle.itemBackgroundColor
            }
//        }
        placeholderTextColor: appStyle.unhighlight(appStyle.textColor)
        selectionColor: appStyle.selectionColor
        selectedTextColor: appStyle.textColor
        textColor: appStyle.textColor
        font.family: appStyle.labelFontFamily
        font.weight: appStyle.labelFontWeight
        font.pixelSize: appStyle.labelFontPixelSize
        renderType: Text.NativeRendering
    }
}
