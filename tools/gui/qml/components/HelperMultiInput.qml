/*******************************************************************************
 *
 * Copyright 2017-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
 *                2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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

//// UI ////
ColumnLayout {
    //// in ////
    property string fieldName

    //// out ////
    property var parameters: ({})

    function beforeOperation() {
        var input = []
        for (var i = 0; i < inputRepeater.model.count; ++i) {
            if (inputRepeater.model.get(i).name) {
                input.push( inputRepeater.model.get(i).name )
            }
        }
        if (entityChooserInput.currentEntityPath !== "") {
            input.push( entityChooserInput.currentEntityPath )
        }

        console.log("parameters: " + parameters);
        console.log("fieldName: " + fieldName);
        console.log("input: " + input);
        parameters[fieldName] = input;
        console.log("parameters: " + parameters);
    }

    GridLayout {
        z: 200
        // input
        Repeater {
            id: inputLabelRepeater
            model: 0
            StyledLabel {
                Layout.column: 0
                Layout.row: 2 + (2 * index)
                text: fieldName + "[" + index + "]"
            }
        }
        Repeater {
            z: 199
            id: inputRepeater
            model: ListModel {}
            EntityChooser {
                Layout.column: 1
                Layout.row: 2 + (2 * index)
                z: 188 - index
                id: entityChooserInputRepeated
                Layout.fillWidth: true
                currentEntityPath: inputRepeater.model.get(index).name
                onCurrentEntityPathChanged: {
                    inputRepeater.model.get(index).name = currentEntityPath
                }
            }
        }

        StyledLabel {
            Layout.column: 0
            Layout.row: 3 + (2 * inputRepeater.count)
            text: fieldName + "[" + inputRepeater.count + "]"
        }
        EntityChooser {
            Layout.column: 1
            Layout.row: 3 + (2 * inputRepeater.count)
            z: 188 - index
            id: entityChooserInput
            Layout.fillWidth: true
            currentEntityPath: root.currentEntityPath
        }
        Connections {
            target: entityChooserInput.internalTextField
            onEditingFinished: {
                if (entityChooserInput.currentEntityPath) {
                    var entityLastName = entityChooserInput.currentEntityPath
                    entityChooserInput.currentEntityPath = ""

                    inputRepeater.model.append( { "name": entityLastName } )
                    inputLabelRepeater.model = inputRepeater.count
                }
            }
        }
    }
}
