import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4

Button {
    style: ButtonStyle {
        label: Label {
            renderType: Text.NativeRendering
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            text: control.text
            color: enabled ? appStyle.textColor : appStyle.textColorDisabled
            font.family: appStyle.labelFontFamily
            font.weight: appStyle.labelFontWeight
            font.pixelSize: appStyle.labelFontPixelSize
        }

        background: Rectangle {
            implicitWidth: 100
            implicitHeight: 25
            color: control.hovered ? appStyle.backgroundHighlightColor : appStyle.backgroundColor
            border.width: control.activeFocus ? 2 : 1
            border.color: appStyle.buttonBorderColor
            smooth: false
            radius: 2
        }
    }
}
