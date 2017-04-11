import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1

Item {
    id: root
    property var currentOperator: listview.model.get(listview.currentIndex)
    ScrollView {
        anchors.fill: parent
        ListView {
            id: listview
            model: ListModel {}
            Component.onCompleted: {
                globalRepository.listOperators().forEach(function(n){listview.model.append(n)})
            }
            delegate: RowLayout {
                Image {
                    width: 24
                    height: 24
                    //source: "qrc://icon/asset-green-24-ns.png"//
                    source: "image://icon/badge-circle-direction-right"
                }
                Text {
                    Layout.fillHeight: true
                    renderType: Text.NativeRendering
                    text: moduleName
                    verticalAlignment: Text.AlignVCenter
                    color: palette.text
                    MouseArea {
                        anchors.fill: parent
                        onClicked: listview.currentIndex = index
                    }
                }
            }
            highlight: Rectangle { color: palette.highlight }
        }
    }
}
