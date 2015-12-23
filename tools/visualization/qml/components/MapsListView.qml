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
        width: mapsList.currentItem.width
        height: mapsList.currentItem.height
        color: palette.highlight
        radius: 1
        y: mapsList.currentItem.y
    }
    highlightFollowsCurrentItem: false
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
            text: Globals.getMap(mapId).name.length===0?"<empty name>":Globals.getMap(mapId).name
            textColor: mapsList.currentIndex == index?palette.highlightedText:palette.text
            style: TextFieldStyle {
                //renderType: Text.NativeRendering

                background: Rectangle {
                    radius: 2
                    border.color: mapsList.currentIndex == index?palette.highlight:palette.dark
                    border.width: 1
                    color: Qt.darker(palette.highlight)
                    opacity: Math.min(innerText.hovered * 0.2 + innerText.focus, 1.0)
                }
            }
            onFocusChanged: {
                if(focus) mapsList.currentIndex = index
            }
            onEditingFinished: {
                //enabled = false
                focus = false
                if(text !== Globals.getMap(mapId).name) return
                var opdesc = {
                    "operatorname":"updatemetadata",
                    "params": [
                        {
                            "key": "target",
                            "mapval": mapId
                        },
                        {
                            "key": "name",
                            "strval": text
                        }
                    ]
                }
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
                height: mapsList.currentIndex === index ? 0 : parent.height // hide mousearea to make cursor change for underlaying textfield
                acceptedButtons: Qt.LeftButton // | Qt.RightButton
                onClicked: mapsList.currentIndex = index
            }
        }
    }
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
