import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Controls.Private 1.0

// This file contains private Qt Quick modules that might change in future versions of Qt
// Tested on: Qt 5.10

Label {
    //Note: If tooltips are enabled, the lable can not be clicked through
    property alias tooltip: tooltipMousearea.text
    color: enabled ? appStyle.textColor : appStyle.textColorDisabled
    font.family: appStyle.labelFontFamily
    font.weight: appStyle.labelFontWeight
    font.pixelSize: appStyle.labelFontPixelSize
    height: appStyle.controlHeightOuter
    verticalAlignment: Label.AlignVCenter
    MouseArea {
        id: tooltipMousearea
        property string text

        anchors.fill: parent
        hoverEnabled: tooltipMousearea.enabled

        acceptedButtons: Qt.NoButton
        onExited: Tooltip.hideText()
        onCanceled: Tooltip.hideText()

        Timer {
            interval: 200
            running: tooltipMousearea.enabled && tooltipMousearea.containsMouse && tooltipMousearea.text.length
            onTriggered: Tooltip.showText(tooltipMousearea, Qt.point(tooltipMousearea.mouseX, tooltipMousearea.mouseY), tooltipMousearea.text)
        }
    }
}
