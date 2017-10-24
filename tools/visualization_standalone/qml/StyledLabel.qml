import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4

Label {
    //Note: If tooltips are enabled, the lable can not be clicked through
    property alias tooltip: button.tooltip
    color: enabled ? appStyle.textColor : appStyle.textColorDisabled
    font.family: appStyle.labelFontFamily
    font.weight: appStyle.labelFontWeight
    font.pixelSize: appStyle.labelFontPixelSize
    height: appStyle.controlHeightOuter
    verticalAlignment: Label.AlignVCenter
    Button {
        visible: tooltip && tooltip != ""
        id: button
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: appStyle.controlHeightOuter

        style: ButtonStyle {
            background: Rectangle {
                border.width: 0
                color: "transparent"
            }
        }
    }
}
