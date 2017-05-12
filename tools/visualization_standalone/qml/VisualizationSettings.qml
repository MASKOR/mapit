import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

Dialog {
    id: root
    width: 300
    height: 400
    modality: Qt.WindowModal
    visible: false
    property var model: ListModel {
        ListElement { label: "Background Color"; propName: "backgroundColor" }
        ListElement { label: "Text Color"; propName: "textColor" }
        ListElement { label: "Text Color Disabled"; propName: "textColorDisabled" }
        ListElement { label: "Selection Color"; propName: "selectionColor" }
        ListElement { label: "Selection Border Color"; propName: "selectionBorderColor" }
        ListElement { label: "Highlight Color"; propName: "highlightColor" }
        ListElement { label: "Background Hilight Color"; propName: "backgroundHighlightColor" }
        ListElement { label: "View Border Color"; propName: "viewBorderColor" }
        ListElement { label: "Button Border Color"; propName: "buttonBorderColor" }
        ListElement { label: "Item Background Color"; propName: "itemBackgroundColor" }
        ListElement { label: "Item Color"; propName: "itemColor" }
        ListElement { label: "Icon Highlight Color"; propName: "iconHighlightColor" }
    }

    onVisibleChanged: {
        if(visible) {
            sizeSlider.value = appStyle.controlHeight
        }
    }

    title: "Visualization Settings"
    contentItem: Rectangle {
        anchors.fill: parent
        color: appStyle.backgroundColor
        ScrollView {
            anchors.fill: parent
            ColumnLayout {
                RowLayout {
                    Layout.fillWidth: true
                    StyledLabel {
                        width: 200
                        text: "Size"
                    }
                    Slider {
                        id: sizeSlider
                        width: 200
                        maximumValue: 50
                        minimumValue: 10
                        Binding {
                            id: theBinding
                            target: appStyle
                            property: "controlHeight"
                            value: sizeSlider.value
                            when: root.visible
                        }
                        style: SliderStyle {
                            groove: Rectangle {
                                implicitWidth: 200
                                implicitHeight: 1
                                color: appStyle.itemBackgroundColor
                            }
                            handle: Rectangle {
                                anchors.centerIn: parent
                                color: control.pressed ? appStyle.itemColor : appStyle.itemBackgroundColor
                                border.color: control.pressed ? appStyle.selectionBorderColor : appStyle.buttonBorderColor
                                border.width: 1
                                implicitWidth: 10
                                implicitHeight: 10
                                radius: 5
                            }
                        }
                    }
                }
                GridLayout {
                    Layout.fillWidth: true
                    columns: 2
                    Repeater {
                        model: root.model
                        StyledLabel {
                            Layout.column: 0
                            Layout.row: index
                            Layout.fillWidth: true
                            text: label
                        }
                    }
                    Repeater {
                        model: root.model
                        StyleColorChooser {
                            Layout.column: 1
                            Layout.row: index
                            stylePropertyName: propName
                            isShowing: root.visible
                        }
                    }
                }
            }
        }
    }
}
