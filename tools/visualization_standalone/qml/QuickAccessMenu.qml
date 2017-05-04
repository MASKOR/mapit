import QtQuick 2.0
import QtQuick.Controls 1.2
import QtQuick.Layouts 1.1
//import "qrc:/qml/theme/";

Item {
    id: root
    property bool allowNew: true
    property var model

    property ListModel filteredModel: ListModel {}
    property alias selection: lv.currentItem
    property alias currentText: ff.text
    signal action(var item)
//    gradient: Gradient {
//        GradientStop { position: 0.0; color: ColorTheme.contextMenu1 }
//        GradientStop { position: 1.0; color: ColorTheme.contextMenu2 }
//    }
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
        for(var i=0; i<root.model.length ; ++i) {
            filteredModel.append({"displayName": model[i]});
        }
    }
//    ColumnLayout {
//        anchors.fill: parent
//        anchors.bottomMargin: 2

        Keys.onReturnPressed: {
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
        TextField {
            //Layout.fillWidth: true
            anchors.fill: parent
            id: ff
            placeholderText: "filter..."
            onTextChanged: {
                filteredModel.clear();
                var re = new RegExp(ff.text, "i"); //Case insensitive
                for(var i=0; i<model.length ; ++i) {
                    var block = model[i];
                    if(block.match(re)) {
                        filteredModel.append({"displayName": block});
                        console.log(block)
                    }
                }
                lv.currentIndex = 0;
                console.log("done!!!")
            }
//        }
    }
    Rectangle {
        visible: lv.focus || ff.focus
        anchors.top: ff.bottom
        anchors.left: ff.left
        anchors.right: ff.right
        color: appStyle.backgroundColor
        border.width: 1
        border.color: "orange"
        height: 200
        z: 1000
        ListView {
            anchors.leftMargin: 3
            anchors.fill: parent
            clip: true
            id: lv
            model: filteredModel
            delegate: Text {
                text: displayName
                color: lv.currentIndex===index?"grey":"black"
                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        lv.currentIndex = index
                        ff.text = displayName
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
        }
    }
}

