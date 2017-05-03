import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Controls.Styles 1.4

Item {
    id: root
    property var currentOperator //: listview.model[listview.currentIndex]
    ScrollView {
        anchors.fill: parent
//        ListView {
//            id: listview
//            model: globalRepository.operators// ListModel {}
////            Component.onCompleted: {
////                globalRepository.listOperators().forEach(function(n){listview.model.append(n)})
////            }
//            delegate: Component {
//                RowLayout {
//                    Image {
//                        width: 24
//                        height: 24
//                        //source: "qrc://icon/asset-green-24-ns.png"//
//                        source: "image://icon/badge-circle-direction-right"
//                    }
//                    Text {
//                        Layout.fillHeight: true
//                        renderType: Text.NativeRendering
//                        text: listview.model[index].moduleName
//                        verticalAlignment: Text.AlignVCenter
//                        color: palette.text
//                        MouseArea {
//                            anchors.fill: parent
//                            onClicked: listview.currentIndex = index
//                        }
//                    }
//                }
//            }
//            highlight: Rectangle { color: palette.highlight }
//        }
        anchors.leftMargin: gridRoot.gridMargin
        GridView {
            property int gridMargin: 10
            property int buttonSize: 80
            id: gridRoot
            clip: true
            topMargin: gridMargin
            cellHeight: buttonSize + gridMargin
            cellWidth: buttonSize + gridMargin
            model: globalRepository.operators
            delegate: Button {
                id: entityButton
                width: gridRoot.buttonSize
                height: gridRoot.buttonSize
                style: ButtonStyle {
                    background: Rectangle {
                        border.width: 0
                        color: Qt.darker("grey")
                    }
                }
                tooltip: qsTr("You can click here show details of <i>%1</i>.").arg(
                             gridRoot.model[index].moduleName)
                Column {
                    anchors.centerIn: parent
                    Image {
                        source: "image://operator/"+gridRoot.model[index].moduleName//"image://icon/badge-circle-direction-right"
                        width: 50
                        fillMode: Image.PreserveAspectFit
                        anchors.horizontalCenter: parent.horizontalCenter
                    }
                    Text {
                        width: gridRoot.buttonSize
                        text: gridRoot.model[index].moduleName
                        anchors.horizontalCenter: parent.horizontalCenter
                        //anchors.bottom: parent.bottom
                        wrapMode: Text.WrapAnywhere
                        horizontalAlignment: Text.AlignHCenter
                        color: Qt.darker("white")
//                        font.family: editorContent.labelFontFamily
//                        font.weight: editorContent.labelFontWeight
                        font.pixelSize: 9
                    }
                }
                MouseArea {
                    width: gridRoot.buttonSize
                    height: gridRoot.buttonSize
                    onClicked: root.currentOperator = gridRoot.model[index]
                }
            }
        }
    }
}
