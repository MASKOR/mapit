import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4

Slider {
    style: SliderStyle {
        groove: Rectangle {
            implicitWidth: 200
            implicitHeight: 1
            color: appStyle.itemBackgroundColor
        }
        handle: Item {
            implicitHeight: appStyle.controlHeightInner
            implicitWidth: appStyle.controlHeightInner*0.5
            Rectangle {
                anchors.centerIn: parent
                color: control.pressed ? appStyle.itemColor : appStyle.itemBackgroundColor
                border.color: control.pressed ? appStyle.selectionBorderColor : appStyle.buttonBorderColor
                border.width: 1
                implicitWidth: appStyle.controlHeightInner*0.66
                implicitHeight: appStyle.controlHeightInner*0.66
                radius: implicitHeight*0.5
            }
        }
    }
}
