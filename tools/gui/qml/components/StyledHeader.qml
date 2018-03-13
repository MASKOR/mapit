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

Rectangle {
    id: root
    property alias text: headerLabel.text
    property bool checked: true
    property bool hasIcon: icon.status == Image.Ready
    property alias iconSource: icon.source
    default property alias _conentChildren: flow.data
    color: appStyle.headerColor
    height: appStyle.controlHeightContainer
    implicitHeight: appStyle.controlHeightContainer
    Image {
        id: icon
        visible: root.hasIcon
        anchors.left: parent.left
        anchors.leftMargin: root.hasIcon ? 8 : 0
        anchors.verticalCenter: parent.verticalCenter
        height: appStyle.iconSize
        width: root.hasIcon ? appStyle.iconSize : 0
        opacity: 0.5
    }

    StyledLabel {
        id: headerLabel
        anchors.left: icon.right
        anchors.leftMargin: 8
        anchors.verticalCenter: parent.verticalCenter
        font.weight: appStyle.headerFontWeight
    }

    Flow {
        id: flow
        z: 10
        height: appStyle.controlHeightOuter
        anchors.left: headerLabel.right
        anchors.leftMargin: 8
        anchors.verticalCenter: parent.verticalCenter
    }
//    Rectangle {
//        z: 9
//        anchors.left: flow.left
//        anchors.right: parent.right
//        anchors.top: parent.top
//        anchors.bottom: parent.bottom
//        //anchors.bottomMargin: 3
//        color: appStyle.unhighlight(root.color)
//        gradient: Gradient {
//            GradientStop { position: 1.0; color: "black" }
//            GradientStop { position: 0.0; color: "grey" }
//        }

//        opacity: 0.2
//    }

    Button {
        anchors.right: parent.right
        anchors.rightMargin: 8
        anchors.verticalCenter: parent.verticalCenter
        style: ButtonStyle {
            padding.top: 0
            padding.bottom: 0
            padding.right: 0
            padding.left: 0
            background: Rectangle {
                border.color: "transparent"
                border.width: 0
                color: "transparent"
            }
        }
        height: appStyle.controlHeightOuter
        width: height
        tooltip: root.checked ? qsTr("Collapse") : qsTr("Expand")

        Image {
            id: visibleImage
            height: appStyle.iconSize
            width: appStyle.iconSize
            anchors.verticalCenter: parent.verticalCenter
            source: "image://material/ic_arrow_drop_down"
            rotation: root.checked ? 0 : 180
            //source: root.checked ? "image://icon/badge-square-direction-down" : "image://icon/badge-square-direction-up"
        }
        onClicked: {
            root.checked = !root.checked
        }
    }
}
