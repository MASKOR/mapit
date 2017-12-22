import QtQuick 2.0
import QtQuick.Controls 1.2
import QtQuick.Layouts 1.1
//import "qrc:/qml/theme/";

Item {
    id: root
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

    property ListModel filteredModel: ListModel {}
    property alias selection: lv.currentItem
    property alias currentText: ff.text
    property var internalTextField: ff
    signal action(var item)

    onModelChanged: reinit()
    onXChanged: reinit()
    onYChanged: reinit()
    onVisibleChanged: reinit()
    z: 1000
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
        onActiveFocusChanged: dropDown.visible = activeFocus
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
            dropDown.visible = activeFocus
        }
    }
    Rectangle {
        visible: false
        id: dropDown

        anchors.top: ff.bottom
        anchors.left: ff.left
        anchors.right: ff.right
        color: appStyle.itemBackgroundColor
        border.width: 1
        border.color: appStyle.selectionBorderColor
        height: 200
        z: 1000

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
            z:10
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
                }
            }
        }
        MouseArea {
            anchors.fill: parent
            onClicked: {
                dropDown.visible = false
            }
        }
    }
}

