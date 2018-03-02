/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

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
