import QtQuick 2.0
import QtQuick.Controls 1.2
import QtQuick.Layouts 1.1
//import "qrc:/qml/theme/";

Rectangle {
    id: root
    property bool allowNew: true
    property ListModel model: ListModel {}

    property ListModel filteredModel: ListModel {}
    property alias selection: lv.currentItem
    signal action(var item)
    border.width: 1
    border.color: "orange"
//    gradient: Gradient {
//        GradientStop { position: 0.0; color: ColorTheme.contextMenu1 }
//        GradientStop { position: 1.0; color: ColorTheme.contextMenu2 }
//    }
    onXChanged: reinit()
    onYChanged: reinit()
    onVisibleChanged: reinit()
    function reinit() {
        if(visible) {
            ff.text = "";
            ff.forceActiveFocus();
        }
        filteredModel.clear();
        for(var i=0; i<root.model.count ; ++i) {
            filteredModel.append(model.get(i));
        }
    }
    ColumnLayout {
        anchors.fill: parent
        anchors.bottomMargin: 2

        Keys.onReturnPressed: {
            root.action(filteredModel.get(lv.currentIndex));
            event.accepted = true;
        }

        Keys.onPressed: {
            if (event.key === Qt.Key_Up) {
                lv.currentIndex = Math.max(0, lv.currentIndex - 1);
                event.accepted = true;
            } else if(event.key === Qt.Key_Down) {
                lv.currentIndex = lv.currentIndex + 1;
                event.accepted = true;
            }
        }
        TextField {
            Layout.fillWidth: true
            id: ff
            placeholderText: "filter..."
            onTextChanged: {
                filteredModel.clear();
                var re = new RegExp(ff.text, "i"); //Case insensitive
                for(var i=0; i<blocksModel.count ; ++i) {
                    var block = blocksModel.get(i);
                    if(block.displayName.match(re)) {
                        filteredModel.append(block);
                    }
                }
                lv.currentIndex = 0;
            }
        }
        ListView {
            clip: true
            Layout.fillWidth: true
            Layout.fillHeight: true
            id: lv
            model: filteredModel
            delegate: Text {
                text: displayName
                color: lv.currentIndex===index?"grey":"black"
                MouseArea {
                    anchors.fill: parent
                    onClicked: lv.currentIndex = index
                    onDoubleClicked: root.createBlock(filteredModel.get(lv.currentIndex))
                }
            }
        }
    }
}

