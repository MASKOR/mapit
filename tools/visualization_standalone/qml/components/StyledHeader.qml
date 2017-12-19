import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4

Rectangle {
    id: root
    property alias text: headerLabel.text
    property bool checked: true
    property bool hasIcon: icon.status == Image.Ready
    property alias iconSource: icon.source
    default property alias _conentChildren: flow.data
    color: appStyle.headerColor
    height: appStyle.controlHeightContainer
    implicitHeight: appStyle.controlHeightContainer
    Image {
        id: icon
        visible: root.hasIcon
        anchors.left: parent.left
        anchors.leftMargin: root.hasIcon ? 8 : 0
        anchors.verticalCenter: parent.verticalCenter
        height: appStyle.iconSize
        width: root.hasIcon ? appStyle.iconSize : 0
        opacity: 0.5
    }

    StyledLabel {
        id: headerLabel
        anchors.left: icon.right
        anchors.leftMargin: 8
        anchors.verticalCenter: parent.verticalCenter
        font.weight: appStyle.headerFontWeight
    }

    Flow {
        id: flow
        height: appStyle.controlHeightOuter
        anchors.topMargin: (appStyle.controlHeightContainer-appStyle.controlHeightOuter) * 0.5
        anchors.left: headerLabel.right
        anchors.leftMargin: 8
        //anchors.verticalCenter: parent.verticalCenter
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
        height: appStyle.controlHeightOuter
        width: height
        tooltip: root.checked ? qsTr("Collapse") : qsTr("Expand")

        Image {
            id: visibleImage
            height: appStyle.iconSize
            width: appStyle.iconSize
            anchors.verticalCenter: parent.verticalCenter
            source: "image://material/ic_arrow_drop_down"
            rotation: root.checked ? 0 : 180
            //source: root.checked ? "image://icon/badge-square-direction-down" : "image://icon/badge-square-direction-up"
        }
        onClicked: {
            root.checked = !root.checked
        }
    }
}
