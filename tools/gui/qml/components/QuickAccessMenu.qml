/*******************************************************************************
 *
 * Copyright 2017-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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
import QtQuick.Controls 1.2
import QtQuick.Layouts 1.1
//import "qrc:/qml/theme/";

Item {
    id: root
    property var popupLayer: appStyle.popupLayerRight
    onPopupLayerChanged: if(popupLayer) dropDown.parent = popupLayer
    property bool allowNew: true
    property var model

//    property var blurMouseArea: globalBlur
    property alias dropDownVisible: dropDown.visible

    implicitHeight: ff.height
//    Connections {
//        target: blurMouseArea
//        onClicked: dropDown.visible = false
//    }
    onFocusChanged: if(!focus) dropDown.visible = false

    Connections {
        target: globalMouseEventFilter
        onMouseReleased: {
            if(dropDownVisible && !doNotBlurTimer.running) {
                blurTimer.start()
            }
        }
    }
    Timer {
        id: blurTimer
        interval: 10
        repeat: false
        onTriggered: {
            dropDownVisible = false
        }
    }
    Timer {
        //HACK: after first click (when popup becomes visible caused by focus
        id: doNotBlurTimer
        interval: 200
        repeat: false
    }

    property ListModel filteredModel: ListModel {}
    property alias selection: lv.currentItem
    property alias currentText: ff.text
    property var internalTextField: ff
    signal action(var item)

    onModelChanged: {
        var oldName = currentText
        reinit()
        currentText = oldName
    }
    onXChanged: reinit()
    onYChanged: reinit()
    onVisibleChanged: reinit()
    //z: 1000
    function reinit() {
        if(visible) {
            ff.text = "";
            ff.forceActiveFocus();
        }
        filteredModel.clear();
        if(root.model) {
            for(var i=0; i<root.model.length ; ++i) {
                filteredModel.append({"displayName": model[i]});
            }
        }
    }
    Keys.onReturnPressed: {
        ff.text = filteredModel.get(lv.currentIndex).displayName
        if(allowNew) {
            root.action(ff.text);
        } else {
            root.action(filteredModel.get(lv.currentIndex).displayName);
        }
        event.accepted = true;
    }

    Keys.onPressed: {
        if (event.key === Qt.Key_Up) {
            lv.currentIndex = Math.max(0, lv.currentIndex - 1);
            event.accepted = true;
        } else if(event.key === Qt.Key_Down) {
            lv.currentIndex = lv.currentIndex + 1;
            event.accepted = true;
        } else if(event.key === Qt.Key_Tab) {
            ff.focus = false
            lv.focus = false
        }
    }
    StyledTextField {
        anchors.left: parent.left
        anchors.right: parent.right
        id: ff
        placeholderText: "filter..."
        focus: false
        onActiveFocusChanged: {
            doNotBlurTimer.start()
            dropDown.visible = activeFocus && filteredModel.count > 0
            blurTimer.stop()
        }
        onTextChanged: {
            filteredModel.clear();
            var re = new RegExp(ff.text, "i"); // Case insensitive
            for(var i=0; i<model.length ; ++i) {
                var block = model[i];
                if(block.match(re)) {
                    filteredModel.append({"displayName": block});
                }
            }
            lv.currentIndex = 0;
            dropDown.visible = activeFocus && filteredModel.count > 0
        }
    }
    Rectangle {
        Component.onCompleted: {
            if(root.popupLayer)
            parent = root.popupLayer
        }
        onVisibleChanged: {
            var pos = ff.mapToItem(parent, 0, ff.height)
            x = pos.x
            y = pos.y
        }

        visible: false
        id: dropDown
        width: ff.width
        height: 200
        color: appStyle.itemBackgroundColor
        border.width: 1
        border.color: appStyle.selectionBorderColor
        //z: 1000

        Component {
            id: highlightBar
            Rectangle {
                width: lv.width
                x: 0
                y: lv.hoveredItem ? lv.hoveredItem.y : 0
                height: lv.hoveredItem ? lv.hoveredItem.height : 0
                color: appStyle.highlightColor
            }
        }

        ListView {
            z: fillingMouseArea.z+1
            anchors.margins: 3
            anchors.fill: parent
            clip: true
            id: lv
            model: filteredModel
            highlight: highlightBar
            highlightFollowsCurrentItem: false
            property var hoveredItem
            delegate: StyledLabel {
                text: displayName
                width: parent.width
                color: overItemMousearea.containsMouse ? appStyle.selectionColor : appStyle.textColor
                //color: lv.currentIndex===index?"grey":"black"
                MouseArea {
                    id: overItemMousearea
                    anchors.fill: parent
                    property var drop: dropDown
                    hoverEnabled: true

                    onClicked: {
                        lv.currentIndex = index
                        ff.text = displayName
                        drop.visible = false
                    }
                    onDoubleClicked: {
                        if(allowNew) {
                            root.action(ff.text);
                        } else {
                            root.action(filteredModel.get(lv.currentIndex).displayName);
                        }
                    }
                }
            }
            MouseArea {
                anchors.fill: parent
                hoverEnabled: true
                acceptedButtons: Qt.NoButton
                onPositionChanged: {
                    var hitem = lv.itemAt(mouseX, mouseY)
                    if(hitem) lv.hoveredItem = hitem
                    blurTimer.stop()
                }
            }
        }
        MouseArea {
            id: fillingMouseArea
            anchors.fill: parent
            onClicked: {
                dropDown.visible = false
                blurTimer.stop()
            }
        }
    }
}

