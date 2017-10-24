import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Layouts 1.1
import QtQuick.Window 2.0

Window {
    id: root
    width: controlsContainer.width
    height: 400
    color: appStyle.backgroundColor
    flags: Qt.Tool
    modality: Qt.NonModal
    visible: false
    maximumWidth: controlsContainer.width
    minimumWidth: controlsContainer.width
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
            sizeSlider.value = appStyle.controlHeightInner
            radiansCheckbox.checked = appStyle.useRadians
            axisCheckbox.checked = appStyle.coordinateSystemYPointsUp
            gridLinesInput.text = appStyle.gridLines
            gridSpacingInput.text = appStyle.gridSpacing
            lodSlider.value = appStyle.pointcloudLod
            camScaleSlider.value = appStyle.cameraScale
        }
    }

    title: "Visualization Settings"
    Rectangle {
        anchors.fill: parent
        color: appStyle.backgroundColor
        ScrollView {
            anchors.fill: parent
            ColumnLayout {
                id: controlsContainer
                RowLayout {
                    anchors.leftMargin: appStyle.controlMargin
                    Layout.fillWidth: true
                    StyledLabel {
                        width: 200
                        text: "UI Size"
                    }
                    Slider {
                        id: sizeSlider
                        width: 200
                        stepSize: 1
                        maximumValue: 50
                        minimumValue: 10
                        Binding {
                            target: appStyle
                            property: "controlHeightInner"
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
                    StyledLabel {
                        width: 50
                        text: appStyle.controlHeightInner + " px"
                    }
                }
                StyledHeader {
                    id: headerSettingsGrid
                    text: "Grid Settings"
                    Layout.fillWidth: true
                }
                GridLayout {
                    anchors.leftMargin: appStyle.controlMargin
                    visible: headerSettingsGrid.checked
                    Layout.fillWidth: true
                    StyledLabel {
                        Layout.column: 0
                        Layout.row: 0
                        Layout.fillWidth: true
                        text: "Grid Lines"
                    }
                    StyledTextField {
                        id: gridLinesInput
                        Layout.column: 1
                        Layout.row: 0
                        validator: IntValidator {}
                        Binding {
                            target: appStyle
                            property: "gridLines"
                            value: gridLinesInput.text
                            when: root.visible
                        }
                    }
                    StyledLabel {
                        Layout.column: 0
                        Layout.row: 1
                        Layout.fillWidth: true
                        text: "Line Spacing (meters)"
                    }
                    StyledTextField {
                        id: gridSpacingInput
                        Layout.column: 1
                        Layout.row: 1
                        validator: DoubleValidator {}
                        Binding {
                            target: appStyle
                            property: "gridSpacing"
                            value: gridSpacingInput.text
                            when: root.visible
                        }
                    }
                }
                StyledHeader {
                    id: headerSettingsCoordinateSystem
                    text: "Coordinate System and 3D Settings"
                    Layout.fillWidth: true
                }
                GridLayout {
                    anchors.leftMargin: appStyle.controlMargin
                    visible: headerSettingsCoordinateSystem.checked
                    Layout.fillWidth: true
                    StyledLabel {
                        Layout.column: 0
                        Layout.row: 0
                        Layout.fillWidth: true
                        text: "Use Radians"
                    }
                    StyledCheckBox {
                        id: radiansCheckbox
                        Layout.column: 1
                        Layout.row: 0
                        Binding {
                            target: appStyle
                            property: "useRadians"
                            value: radiansCheckbox.checked
                            when: root.visible
                        }
                    }
                    StyledLabel {
                        Layout.column: 0
                        Layout.row: 1
                        Layout.fillWidth: true
                        text: "Y-AxisPoints up"
                    }
                    StyledCheckBox {
                        id: axisCheckbox
                        Layout.column: 1
                        Layout.row: 1
                        Binding {
                            target: appStyle
                            property: "coordinateSystemYPointsUp"
                            value: axisCheckbox.checked
                            when: root.visible
                        }
                    }
                    StyledLabel {
                        Layout.column: 0
                        Layout.row: 2
                        Layout.fillWidth: true
                        text: "Level of Detail (Pointclouds)"
                    }
                    StyledSlider {
                        id: lodSlider
                        Layout.column: 1
                        Layout.row: 2
                        minimumValue: 0.01
                        maximumValue: 20.0
                        Binding {
                            target: appStyle
                            property: "pointcloudLod"
                            value: lodSlider.value
                            when: root.visible
                        }
                    }
                    StyledLabel {
                        Layout.column: 0
                        Layout.row: 3
                        Layout.fillWidth: true
                        text: "Camera Scale"
                    }
                    StyledSlider {
                        id: camScaleSlider
                        Layout.column: 1
                        Layout.row: 3
                        minimumValue: 1.0
                        maximumValue:  20.0
                        Binding {
                            target: appStyle
                            property: "cameraScale"
                            value: camScaleSlider.value
                            when: root.visible
                        }
                    }
                }
                StyledHeader {
                    id: headerSettingsUi
                    text: "UI Settings"
                    Layout.fillWidth: true
                    checked: false
                }
                GridLayout {
                    anchors.leftMargin: appStyle.controlMargin
                    visible: headerSettingsUi.checked
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
                            id: colorChooser
                            Layout.column: 1
                            Layout.row: index
                            stylePropertyName: propName
                            isShowing: root.visible
                            Connections {
                                target: appStyle
                                onThemeChanged: colorChooser.reload()
                            }
                        }
                    }
                }
                RowLayout {
                    StyledButton {
                        text: "Reset colors Dark"
                        onClicked: appStyle.resetDefaultColorsDark();
                    }
                    StyledButton {
                        text: "Reset colors System"
                        onClicked: appStyle.resetDefaultColorsSystem();
                    }
                }
            }
        }
    }
}
