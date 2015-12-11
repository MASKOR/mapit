import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1

import "."

ListView {
    id: mapsList
    clip: true
    model: Globals.mapIdsModel
    highlight: Rectangle {
        width: mapsList.currentItem.width + 2
        height: mapsList.currentItem.height
        color: palette.highlight
        radius: 1
        y: mapsList.currentItem.y
    }
    highlightFollowsCurrentItem: false
    delegate: Text {
        renderType: Text.NativeRendering
        text: Globals.getMap(mapId).name.length===0?"<empty name>":Globals.getMap(mapId).name
        color: mapsList.currentIndex == index?palette.highlightedText:palette.text
        MouseArea {
            anchors.fill: parent
            onClicked: mapsList.currentIndex = index
        }
    }
    SystemPalette {
        id: palette
    }
}
