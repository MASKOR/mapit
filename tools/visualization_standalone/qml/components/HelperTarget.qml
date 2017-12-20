import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

import ".."
import "../components"

RowLayout {
    property alias text: label.text
    // second part ensures, that no top level entities can be created. This ensures compatibility to tfs.
    property bool valid: entityChooser.currentEntityPath != "" && entityChooser.currentEntityPath.substring(1).indexOf('/') != -1
    property alias currentEntityPath: entityChooser.currentEntityPath
    property string nameOverwrite: ""
    Layout.fillWidth: true
    z: 100
    StyledLabel {
        id: label
        text: nameOverwrite == "" ? "Target:" : nameOverwrite
    }
    EntityChooser {
        id: entityChooser
        Layout.fillWidth: true
    }
}
