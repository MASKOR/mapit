import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Controls.Styles 1.4
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1

import "."

ListView {
    property var mapId
    property var currentLayerId
    id: mapLayers
    clip: true
    model: Globals.getMap(mapId).layers
    highlight: Rectangle {
        width: mapLayers.currentItem.width + 2
        height: mapLayers.currentItem.height
        color: palette.highlight
        radius: 1
        y: mapLayers.currentItem.y
    }
    highlightFollowsCurrentItem: false
//    delegate: Text {
//        renderType: Text.NativeRendering
//        text: mapLayers.model[index].name.length===0?"<empty name>":mapLayers.model[index].name
//        color: mapLayers.currentIndex == index?palette.highlightedText:palette.text
//        MouseArea {
//            anchors.fill: parent
//            onClicked: mapLayers.currentIndex = index
//        }
//    }

    delegate: Item {
        id: theItem
        height: innerText.height
        width: textForWidth.width + 10
        Text {
            id: textForWidth
            opacity: 0.0
            text: innerText.text
            height: 1
        }
        TextField {
            id: innerText
            anchors.fill: parent
            text: mapLayers.model[index].name.length===0?"<empty name>":mapLayers.model[index].name
            textColor: mapLayers.currentIndex == index?palette.highlightedText:palette.text
            style: TextFieldStyle {
                //renderType: Text.NativeRendering

                background: Rectangle {
                    radius: 2
                    border.color: mapLayers.currentIndex === index?palette.highlight:palette.dark
                    border.width: 1
                    color: Qt.darker(palette.highlight)
                    opacity: Math.min(innerText.hovered * 0.2 + innerText.focus, 1.0)
                }
            }
            onFocusChanged: {
                if(focus) mapLayers.currentIndex = index
            }
            onEditingFinished: {
                //enabled = false
                focus = false
                var allLayers = Globals.getMap(mapLayers.mapId).layers;
                for(var i=0 ; i< allLayers.length ; ++i) {
                    var l = allLayers[i]
                    if(l.id === mapLayers.currentLayerId && l.name === text) {
                        return
                    }
                }
                var opdesc = {
                    "operatorname":"updatemetadata",
                    "params": [
                        {
                            "key": "target",
                            "mapval": mapLayers.mapId,
                            "layerval": mapLayers.currentLayerId
                        },
                        {
                            "key": "name",
                            "strval": text
                        }
                    ]
                }
                console.log("dbg: newname" + text);
                console.log("dbg: mapid" + mapLayers.mapId + " lay " + mapLayers.currentLayerId);
                var success = Globals.doOperation( opdesc, function(result) {
                    //enabled = true
                    if(result.status !== 0) {
                        var errMsg = "An operation returned an error.\n";
                        errMsg += "See the log file for more information.\n";
                        errMsg += "\n"
                        errMsg += "(Code: "+result.status+")"
                        errorDialog.showError(errMsg);
                        return
                    }
                    //root.operationResult = result
                    //root.operationFinished(result)
                });
            }
            MouseArea {
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right
                height: mapLayers.currentIndex === index ? 0 : parent.height // hide mousearea to make cursor change for underlaying textfield
                acceptedButtons: Qt.LeftButton // | Qt.RightButton
                onClicked: mapLayers.currentIndex = index
            }
        }
    }



    onCurrentIndexChanged: {
        var layerId = mapLayers.model[currentIndex].id
        if(layerId !== currentLayerId) {
            currentLayerId = layerId
        }
    }
    onCurrentLayerIdChanged: {
        for(var i=0 ; i < mapLayers.model.count ; ++i) {
            var layerId = mapLayers.model[i].id
            if( layerId === currentLayerId ) {
                currentIndex = i;
                return
            }
        }
    }
    Dialog {
        id: errorDialog
        property alias text: errorText.text
        title: "Error"
        standardButtons: StandardButton.Ok
        visible: false
        Text {
            id: errorText
            renderType: Text.NativeRendering
            color: palette.text
        }
        function showError(msg) {
            text = msg
            visible = true
        }
    }
    SystemPalette {
        id: palette
    }
}