import QtQuick 2.5
import QtQuick.Controls 1.4
import QtQuick.Dialogs 1.2

StyledButton {
    id: root
    property string stylePropertyName
    property bool isShowing
    useHoverHighlight: false
    function reload() {
        if(isShowing) {
            colorDialog.originalColor = appStyle[root.stylePropertyName]
            colorDialog.shownColor = colorDialog.originalColor
        }
    }

    Binding {
        id: theBinding
        target: appStyle
        property: root.stylePropertyName
        value: colorDialog.shownColor
        when: colorDialog.visible
    }
    onIsShowingChanged: reload()

    color: colorDialog.shownColor
    onClicked: colorDialog.open()
    ColorDialog {
        id: colorDialog
        property color originalColor
        property color shownColor
        modality: Qt.ApplicationModal
        onVisibleChanged: {
            if(visible) {
                originalColor = appStyle[root.stylePropertyName]
                shownColor = originalColor
                color = shownColor
                currentColor = shownColor
            }
        }
        onCurrentColorChanged: shownColor = currentColor
        onAccepted: shownColor = color
        onRejected: shownColor = originalColor
    }
}
