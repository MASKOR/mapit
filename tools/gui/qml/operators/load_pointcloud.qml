/*******************************************************************************
 *
 * Copyright 2017-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

ColumnLayout {
    id: root
    //// in ////
    property bool editable
    property string currentEntityPath

    function fromParameters(params) {
        fileChooser.filename = params.filename
        entityChooser.currentEntityPath = params.target
    }

    //// out ////
    property bool valid:    fileChooser.valid
    property var parameters: {
        "filename": fileChooser.filename,
        "target": entityChooser.currentEntityPath,
        "frame_id": frameId.currentText,
        "sec":      parseInt(stampSec.text),
        "nsec:":    parseInt(stampNSec.text)
    }

    //// UI ////
    HelperOpenFile {
        Layout.fillWidth: true
        id: fileChooser
        fileExtension: "pcd"
    }
    HelperTarget {
        Layout.fillWidth: true
        id: entityChooser
        currentEntityPath: root.currentEntityPath
    }
    RowLayout {
        StyledLabel {
            text: "frame_id:"
        }
        FrameIdChooser {
            Layout.fillWidth: true
            Layout.minimumWidth: 100
            id: frameId
            allowNew: true
            currentWorkspace: root.currentWorkspace
        }
    }
    RowLayout {
        StyledLabel {
            text: "Stamp:"
        }
        StyledTextField {
            id: stampSec
            validator: IntValidator {}
            placeholderText: "sec"
        }
        StyledTextField {
            Layout.fillWidth: true
            id: stampNSec
            validator: IntValidator {}
            placeholderText: "nsec"
        }
    }
}
