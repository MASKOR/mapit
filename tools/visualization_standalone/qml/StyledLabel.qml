import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4

Label {
    property alias tooltip: button.tooltip
    color: enabled ? appStyle.textColor : appStyle.textColorDisabled
    font.family: appStyle.labelFontFamily
    font.weight: appStyle.labelFontWeight
    font.pixelSize: appStyle.labelFontPixelSize
    Button {
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
