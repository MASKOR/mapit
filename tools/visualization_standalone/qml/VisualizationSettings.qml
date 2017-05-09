import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Dialogs 1.2

Dialog {
    visible: true
    title: "Visualization Settings"

    contentItem: Rectangle {
        color: appStyle.backgroundColor
        implicitWidth: 400
        implicitHeight: 100
        Text {
            text: "Hello blue sky!"
            color: "navy"
            anchors.centerIn: parent
        }
    }
}
