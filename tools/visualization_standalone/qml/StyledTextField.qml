import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4

TextField {
    style: TextFieldStyle {
        background: Rectangle {
            radius: appStyle.radius
            implicitWidth: appStyle.controlHeight*4
            implicitHeight: appStyle.controlHeight
            border.color: appStyle.buttonBorderColor
            border.width: 1
            color: appStyle.itemBackgroundColor
        }
        placeholderTextColor: appStyle.unhighlight(appStyle.textColor)
        selectionColor: appStyle.selectionColor
        selectedTextColor: appStyle.textColor
        textColor: appStyle.textColor
        font.family: appStyle.labelFontFamily
        font.weight: appStyle.labelFontWeight
        font.pixelSize: appStyle.labelFontPixelSize
        renderType: Text.NativeRendering
    }
}
