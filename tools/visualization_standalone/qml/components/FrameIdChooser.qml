import QtQuick 2.0
import QtQuick.Controls 1.1
import fhac.upns 1.0
import QtQuick.Controls.Styles 1.4

import "."

QuickAccessMenu {
    property var currentCheckout: globalApplicationState.currentCheckout
    z:100
    id: frameIdInput
    height: appStyle.controlHeightInner
    model: currentCheckout?currentCheckout.getFrameIds():[]
}
