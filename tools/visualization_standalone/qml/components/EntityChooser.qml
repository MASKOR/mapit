import QtQuick 2.0
import QtQuick.Controls 1.1
import fhac.upns 1.0
import QtQuick.Controls.Styles 1.4

import "."

Item {
    id:root
    height: appStyle.controlHeightOuter
    z: 1000
    property var currentCheckout: globalApplicationState.currentCheckout
    property alias currentEntityPath: filter.currentText
    property real extendedHeight: 200
    property bool allowNewMap: true
    QuickAccessMenu {
        id: filter
        anchors.fill: parent
        allowNew: true
        model: currentCheckout.entities
    }
}
