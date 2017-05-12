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
    //height: appStyle.controlHeight
    Button {
        visible: tooltip && tooltip != ""
        id: button
        anchors.fill: parent
        style: ButtonStyle {
            background: Rectangle {
                border.width: 0
                color: "transparent"
            }
        }
    }
}
