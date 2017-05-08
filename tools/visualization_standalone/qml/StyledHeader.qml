import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4

Rectangle {
    id: root
    property alias text: headerLabel.text
    property bool checked: true
    color: appStyle.headerColor
    height: 25
    StyledLabel {
        id: headerLabel
        anchors.left: parent.left
        anchors.leftMargin: 8
        anchors.verticalCenter: parent.verticalCenter
        font.weight: appStyle.headerFontWeight
    }
    Button {
        anchors.right: parent.right
        anchors.rightMargin: 8
        anchors.verticalCenter: parent.verticalCenter
        style: ButtonStyle {
            padding.top: 0
            padding.bottom: 0
            padding.right: 0
            padding.left: 0
            background: Rectangle {
                border.color: "transparent"
                border.width: 0
                color: "transparent"
            }
        }
        height: 24
        width: 24
        tooltip: qsTr("Reset Transform")

        Image {
            id: visibleImage
            anchors.verticalCenter: parent.verticalCenter
            source: "image://icon/ic_arrow_drop_down_black"
            rotation: root.checked ? 0 : 180
            //source: root.checked ? "image://icon/badge-square-direction-down" : "image://icon/badge-square-direction-up"
        }
        onClicked: {
            root.checked = !root.checked
        }
    }
}
