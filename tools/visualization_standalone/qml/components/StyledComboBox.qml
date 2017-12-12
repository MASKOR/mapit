import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4

ComboBox {
    style: ComboBoxStyle {
        background: Rectangle {
            implicitHeight: appStyle.controlHeightOuter
            implicitWidth: appStyle.controlHeightOuter*4
            color: appStyle.itemBackgroundColor
            border.color: appStyle.buttonBorderColor
            border.width: 1
            radius: appStyle.radius
        }
        textColor: appStyle.textColor
        selectionColor: appStyle.highlightColor
        selectedTextColor: appStyle.selectionColor
    }
}
