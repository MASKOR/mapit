import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

import ".."

RowLayout {
    property var dialogRoot
    property bool valid: entityChooser.currentEntityPath != ""
    property alias currentEntityPath: entityChooser.currentEntityPath
    Layout.fillWidth: true
    z: 100
    StyledLabel {
        text: "Target:"
    }
    EntityChooser {
        id: entityChooser
        Layout.fillWidth: true
        currentCheckout: dialogRoot.currentCheckout
        //currentEntityPath: dialogRoot.currentEntityPath
    }
}
