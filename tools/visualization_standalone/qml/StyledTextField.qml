import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4

TextField {
    font.pixelSize: appStyle.labelFontPixelSize
    verticalAlignment: TextInput.AlignBottom
    implicitHeight: appStyle.controlHeightOuter
    height: appStyle.controlHeightOuter
    style: TextFieldStyle {
        id: textFieldStyle
        background: //Rectangle {
//            color: "transparent"
//            implicitHeight: appStyle.controlHeightOuter + appStyle.labelFontPixelSize*0.37 // hack for wrong vertical text alignment
//            implicitWidth: appStyle.controlHeightOuter*4
            Rectangle {
                anchors.top: parent.top
                anchors.left: parent.left
                anchors.right: parent.right
                radius: appStyle.radius
                implicitHeight: appStyle.controlHeightOuter
                height: appStyle.controlHeightOuter
                border.color: appStyle.buttonBorderColor
                border.width: 1
                color: appStyle.itemBackgroundColor
            }
//        }
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
