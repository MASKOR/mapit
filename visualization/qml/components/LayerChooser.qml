import QtQuick 2.0
import QtQuick.Controls 1.1
import fhac.upns 1.0
import QtQuick.Controls.Styles 1.4

import "."

Item {
    //TODO: dows not work yet. TODO: Layout Voxelgridfilter dialog.
    id: root
    SystemPalette {
        id: palette
    }
    property var rootMap
    property string choosenLayerId /// can be 0 if new map wants to be created
    property alias choosenLayerName: comboButton.text
    property real extendedHeight: 200
    property bool allowNewLayer: true
    function getIndexWithId(id) {
        for(var i=0 ; i < rootMap.layers.count ; ++i) {
            var curLayerId = rootMap.layers[i].id
            if( id === curLayerId) {
                return i
            }
        }
        return -1
    }
    function getLayerByIndex(index) {
        return rootMap.layers[index]
    }
    function getLayerWithId(id) {
        return getLayerByIndex(getIndexWithId(id))
    }
    function getLayerByName(index) {
        for(var i=0 ; i < root.rootMap.layers.count ; ++i) {
            var curLayer = getLayerByIndex[i]
            if( curLayer.name === text) {
                return curLayer
            }
        }
        return 0
    }
    onChoosenLayerIdChanged: {
        if(choosenLayerId === "" || parseInt(root.choosenLayerId) === 0) return;
        mapLayers.noUpdate = true
        mapLayers.currentIndex = getIndexWithId(root.choosenLayerId)
        mapLayers.noUpdate = false
        var layer = getLayerWithId(root.choosenLayerId)
        if( layer ) {
            comboButton.text = getLayerWithId(root.choosenLayerId).name
        } else {
            //TODO: remove this if/else. This else is executed e.g. when map and layerIds do not fit (map changed, layer not changed yet)
        }
    }
    TextField {
        id: comboButton
        readOnly: !root.allowNewLayer
        textColor: (root.choosenLayerId !== "" && parseInt(root.choosenLayerId) !== 0)?palette.text:"green"
        onTextChanged: {
            // check if choosenId is aleady matching
            if(root.choosenLayerId !== "" && parseInt(root.choosenLayerId) !== 0) {
                var nam = getLayerWithId(root.choosenLayerId).name
                if( nam === text) {
                    return
                }
            }
            // choose first map with given name or make choosenId empty
            var firstLayerWithSameName = root.getLayerByName(text)
            if(firstLayerWithSameName) {
                root.choosenLayerId = firstLayerWithSameName.id;
            } else {
                // no map with the name found...
                root.choosenLayerId = "";
            }
        }
    }

    ListView {
        id: mapLayers
        height: 120
        width: Math.max(parent.width, extendedHeight)
        anchors.top: comboButton.bottom
        clip:true
        visible: comboButton.focus
        property bool noUpdate: false
        onCurrentIndexChanged: {
            if(!noUpdate) {
                root.choosenLayerId = root.getLayerByIndex(currentIndex).id
            }
        }
        model: root.rootMap.layers.count
        highlight: Rectangle {
            width: mapLayers.currentItem.width + 2
            height: mapLayers.currentItem.height
            color: palette.highlight
            radius: 1
            y: mapLayers.currentItem.y
        }
        highlightFollowsCurrentItem: false
        delegate: Text {
            renderType: Text.NativeRendering
            text: root.getLayerByIndex(index).name.length===0?"<empty name>":root.getLayerByIndex(index).name
            color: mapLayers.currentIndex === index?palette.highlightedText:palette.text
            MouseArea {
                anchors.fill: parent
                onClicked: {
                    mapLayers.currentIndex = index
                }
            }
        }
    }
}
