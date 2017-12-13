import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4

Button {
    id: root
    property bool isIcon: false
    property string iconSource
    property color color: appStyle.itemBackgroundColor
    property bool useHoverHighlight: true
    implicitHeight: appStyle.controlHeightOuter
    style: ButtonStyle {
        label: Label {
            visible: !control.isIcon
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
            implicitWidth: control.isIcon ? appStyle.controlHeightOuter : appStyle.controlHeightOuter*4.0
            implicitHeight: appStyle.controlHeightOuter
            color: control.checked
                   ? appStyle.backgroundHighlightColor
                   : control.hovered && control.useHoverHighlight
                     ? appStyle.backgroundHighlightColor
                     : control.isIcon
                       ? "transparent"
                       : control.color
            border.width: control.activeFocus ? 2 : 1
            border.color: control.isIcon ? "transparent" : appStyle.buttonBorderColor
            smooth: false
            radius: appStyle.radius
            Image {
                id: img
                mipmap: false
                smooth: false
                sourceSize: Qt.size(appStyle.iconSize, appStyle.iconSize)
                width: appStyle.iconSize
                height: appStyle.iconSize
                x: 0
                y: 0
                anchors.centerIn: parent
                visible: control.isIcon
                opacity: control.enabled ? 1.0 : 0.5
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                source: control.iconSource && control.iconSource !== "" ? control.iconSource : "image://icon/"
            }
        }
    }
}
