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
                checked: appStyle.currentOperatorListView === 0
                enabled: !checked
                onCheckedChanged: {
                    if(checked) viewModuleButton.checked = false
                }
//                Binding {
//                    target: appStyle
//                    property: "currentOperatorListView"
//                    value: viewListButton.checked ? 1 : 0
//                }
            }
            StyledButton {
                id: viewModuleButton
                isIcon: true
                iconSource: "image://material/ic_view_module"
                checkable: true
                checked: appStyle.currentOperatorListView === 1
                enabled: !checked
                onCheckedChanged: {
                    if(checked) viewListButton.checked = false
                }
            }
        }
        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            StyledLabel {
                anchors.centerIn: parent
                visible: !globalRepository.isLoaded
                text: "No Repository loaded"
            }
            ScrollView {
                visible: viewListButton.checked && globalRepository.isLoaded
                anchors.fill: parent
                ListView {
                    id: listview
                    model: globalRepository.operators
                    delegate: Component {
                        RowLayout {
                            height: appStyle.controlHeightOuter
                            Item {
                                id: iconMarginItem
                                property real generatedWidth: appStyle.iconSize//*(16.0/9.0)
                                width: generatedWidth
                                Image {
                                    sourceSize: Qt.size(iconMarginItem.generatedWidth,appStyle.iconSize)
                                    width: iconMarginItem.generatedWidth
                                    height: appStyle.iconSize
                                    source: "image://operator/"+gridRoot.model[index].moduleName
                                    fillMode: Image.PreserveAspectFit
                                    anchors.verticalCenter: parent.verticalCenter
                                    smooth: false
                                    mipmap: true
                                }
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
                horizontalScrollBarPolicy: Qt.ScrollBarAlwaysOff
                verticalScrollBarPolicy: Qt.ScrollBarAlwaysOff
                anchors.fill: parent
                anchors.margins: gridRoot.gridMargin
                GridView {
                    property int maximumButtonSize: 85
                    property int gridMargin: 5
                    property int columnCount: Math.ceil(gridRoot.width/(gridMargin+maximumButtonSize))
                    property int buttonSize: (gridRoot.width/columnCount)-((columnCount-1)*gridMargin)
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
                                color: appStyle.itemBackgroundColor
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
                                width: gridRoot.buttonSize*0.6
                                height: gridRoot.buttonSize*0.6
                                sourceSize: Qt.size(width, height)
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
