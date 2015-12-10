import QtQuick 2.0
import QtQuick.Controls 1.1
import fhac.upns 1.0
import QtQuick.Controls.Styles 1.4

import "."

Item {
    id:root
    SystemPalette {
        id: palette
    }
    property string choosenMapId /// can be 0 if new map wants to be created
    property alias choosenMapName: comboButton.text
    property real extendedHeight: 200
    property bool allowNewMap: true
    onChoosenMapIdChanged: {
        if(choosenMapId === "" || parseInt(choosenMapId) === 0) return;
        mapsList.noUpdate = true
        mapsList.currentIndex = Globals.mapIdsModel.indexOfMap(choosenMapId)
        mapsList.noUpdate = false
        comboButton.text = Globals.getMap(choosenMapId).name
    }
    TextField {
        id: comboButton
        readOnly: !root.allowNewMap
        textColor: (choosenMapId !== "" && parseInt(root.choosenMapId) !== 0)?palette.text:"green"
        onTextChanged: {
            // check if choosenId is aleady matching
            if(choosenMapId !== "" && parseInt(root.choosenMapId) !== 0) {
                var nam = Globals.getMap(root.choosenMapId).name
                if( nam === text) {
                    return
                }
            }
            // choose first map with given name or make choosenId empty
            for(var i=0 ; i < Globals.mapIdsModel.count ; ++i) {
                var curMapId = Globals.mapIdsModel.get(i).mapId
                var nam2 = Globals.getMap(curMapId).name
                if( nam2 === text) {
                    root.choosenMapId = curMapId;
                    return
                }
            }
            // no map with the name found...
            root.choosenMapId = "";
        }
    }

    ListView {
        id: mapsList
        height: 120
        width: Math.max(parent.width, extendedHeight)
        anchors.top: comboButton.bottom
        clip:true
        visible: comboButton.focus
        property bool noUpdate: false
        onCurrentIndexChanged: {
            if(!noUpdate) {
                root.choosenMapId = Globals.mapIdsModel.get(currentIndex).mapId
            }
        }
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
            text: Globals.getMap(mapId).name
            color: mapsList.currentIndex == index?palette.highlightedText:palette.text
            MouseArea {
                anchors.fill: parent
                onClicked: {
                    mapsList.currentIndex = index
                }
            }
        }
    }
}
