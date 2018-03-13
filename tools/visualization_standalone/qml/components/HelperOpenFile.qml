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

import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

import ".."
import "../components"

RowLayout {
    Layout.fillWidth: true
    property string fileExtension
    property alias filename: fileName.text
    property bool valid: filename != ""
    StyledTextField {
        id: fileName
    }
    StyledButton {
        text: "Open"
        onClicked: {
            openFileDialog.open()
        }
    }
    FileDialog {
        id: openFileDialog
        title: "Open " + fileExtension
        selectExisting: true
        selectFolder: false
        selectMultiple: false
        onAccepted: {
            var filename = fileUrl.toString()
            filename = filename.replace(/^(file:\/{2})/,"")
            fileName.text = filename
        }
    }
}
