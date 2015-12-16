import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Controls.Styles 1.4
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1

import "."

ListView {
    property var currentMapId
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
    delegate: //Item {
//        height: innerText.height
//        width: innerText.width
        TextField {
            id: innerText
            //renderType: Text.NativeRendering
            text: Globals.getMap(mapId).name.length===0?"<empty name>":Globals.getMap(mapId).name
            textColor: mapsList.currentIndex == index?palette.highlightedText:palette.text
            style: TextFieldStyle {
                //textColor: (root.isInputBlock||root.isOutputBlock)?ColorTheme.inputOutputBlockTextColor:ColorTheme.blockTextColor
                renderType: Text.NativeRendering
                background: Rectangle {
                    radius: 2
                    //implicitWidth: middleWidthText.width + 10
                    implicitHeight: 24
                    border.color: mapsList.currentIndex == index?palette.highlight:palette.dark
                    border.width: 1
                    color: palette.highlight
                    opacity: Math.min(innerText.hovered * 0.2 + innerText.focus, 1.0)
                }
            }
            MouseArea {
                anchors.fill: parent
                onClicked: mapsList.currentIndex = index
            }
        }
//        TextField {
//            id: innerText
//            renderType: Text.NativeRendering
//            text: Globals.getMap(mapId).name.length===0?"<empty name>":Globals.getMap(mapId).name
//            color: mapsList.currentIndex == index?palette.highlightedText:palette.text
//            MouseArea {
//                anchors.fill: parent
//                onClicked: mapsList.currentIndex = index
//            }
//        }
//    }
    onCurrentIndexChanged: {
        // simply using currentItem.mapId does not work
        var mapId = Globals.mapIdsModel.get(currentIndex).mapId;
        if(currentMapId !== mapId) {
            currentMapId = mapId
        }
    }
    onCurrentMapIdChanged: {
        for(var i=0 ; i < Globals.mapIdsModel.count ; ++i) {
            var mapId = Globals.mapIdsModel.get(i)
            if( mapId === currentMapId ) {
                currentIndex = i;
                return
            }
        }
    }
    SystemPalette {
        id: palette
    }
}
