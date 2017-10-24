import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4

CheckBox {
    property var overrideControlSize: false
    style: CheckBoxStyle {
        indicator:
//        Rectangle {
//            implicitWidth: overrideControlSize ? 1 : appStyle.controlHeight
//            implicitHeight: overrideControlSize ? 1 : appStyle.controlHeight
//            color: "transparent"
            Rectangle {
                anchors.centerIn: parent
                implicitWidth: overrideControlSize ? 1 : appStyle.controlHeightInner
                implicitHeight: overrideControlSize ? 1 : appStyle.controlHeightInner
                color: control.checked
                       ? appStyle.backgroundHighlightColor
                       : control.hovered
                         ? appStyle.backgroundHighlightColor
                         : appStyle.itemBackgroundColor
                border.width: control.activeFocus ? 2 : 1
                border.color: appStyle.buttonBorderColor
                smooth: false
                radius: appStyle.radius
                Rectangle {
                    visible: control.checked
                    color: appStyle.selectionColor
                    border.color: appStyle.selectionBorderColor
                    radius: 1
                    anchors.margins: 3
                    anchors.fill: parent
                }
            }
//        }
    }
}
