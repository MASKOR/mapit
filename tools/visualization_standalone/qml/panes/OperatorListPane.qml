import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Controls.Styles 1.4
import ".."

Item {
    id: root
    property var currentOperator
    ColumnLayout {
        anchors.fill: parent
        RowLayout {
            Layout.fillWidth: true
            StyledButton {
                id: viewListButton
                isIcon: true
                iconSource: "image://material/ic_view_list"
                checkable: true
                enabled: !checked
                onCheckedChanged: if(checked) viewModuleButton.checked = false
            }
            StyledButton {
                id: viewModuleButton
                isIcon: true
                iconSource: "image://material/ic_view_module"
                checkable: true
                checked: true
                enabled: !checked
                onCheckedChanged: if(checked) viewListButton.checked = false
            }
        }
        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            ScrollView {
                visible: viewListButton.checked
                anchors.fill: parent
                ListView {
                    id: listview
                    model: globalRepository.operators
                    delegate: Component {
                        RowLayout {
                            height: appStyle.controlHeight
                            Image {
                                sourceSize: Qt.size(appStyle.iconSize,appStyle.iconSize)
                                width: appStyle.iconSize
                                height: appStyle.iconSize
                                source: "image://operator/"+gridRoot.model[index].moduleName
                                fillMode: Image.PreserveAspectFit
                                anchors.verticalCenter: parent.verticalCenter
                                smooth: false
                                mipmap: true
                            }
                            StyledLabel {
                                Layout.fillHeight: true
                                renderType: Text.NativeRendering
                                text: listview.model[index].moduleName
                                verticalAlignment: Text.AlignVCenter
                                MouseArea {
                                    anchors.fill: parent
                                    onClicked: listview.currentIndex = index
                                }
                            }
                        }
                    }
                    onCurrentIndexChanged: root.currentOperator = model[currentIndex]
                    highlight: Rectangle { color: appStyle.itemBackgroundColor }
                }
            }
            ScrollView {
                visible: viewModuleButton.checked
                anchors.fill: parent
                anchors.leftMargin: gridRoot.gridMargin
                GridView {
                    property int gridMargin: 5
                    property int buttonSize: 85
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
                                property bool selected: root.currentOperator ? root.currentOperator.moduleName === gridRoot.model[index].moduleName : false
                                border.width: selected ? 1 : 0
                                border.color: appStyle.selectionBorderColor
                                color: Qt.darker("grey")
                            }
                        }
                        tooltip: qsTr("Show details of <i>%1</i>.").arg(
                                     gridRoot.model[index].moduleName)
                        Column {
                            y: 8
                            anchors.horizontalCenter: parent.horizontalCenter
                            Image {
                                anchors.topMargin: 8
                                source: "image://operator/"+gridRoot.model[index].moduleName
                                width: 64
                                height: 64
                                fillMode: Image.PreserveAspectFit
                                anchors.horizontalCenter: parent.horizontalCenter
                                smooth: false
                                mipmap: true
                            }
                            Text {
                                width: gridRoot.buttonSize
                                height: 8
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
    }
}
