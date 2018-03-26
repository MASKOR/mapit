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

import ".."
import "../operators"
import "../components"

ColumnLayout {
    id: root
    //height: implicitHeight

    //// in ////
    property bool shown
    property bool editable
    property var currentWorkspace
    property string currentEntityPath

    function fromParameters(params) {
        entityChooser.currentEntityPath = params.target
        primitiveOption.checkCurrentType(params.type)
    }
    function updateTranslationMatrix() {
        var translateMatrix = Qt.matrix4x4()
        if(xInp == null || yInp == null || zInp == null) return // Note: the line below produced error logs. bug?
        translateMatrix.translate(xInp.text, yInp.text, zInp.text)
        appStyle.tmpPreviewMatrix = translateMatrix
    }
    function execute() {
        if(executeButton.enabled) {
            currentWorkspace.doOperation("load_primitive", parameters)
            currentWorkspace.doOperation("load_tfs", parametersLoadTfs)
        }
    }

    //// out ////
    property bool valid: primitiveOption.getCurrentType() !== "" && entityChooser.valid
    property var parameters: {
        "type": primitiveOption.getCurrentType(),
        "target": entityChooser.currentEntityPath,
        "frame_id": childFrameIdInput.currentText
    }
    property var parametersLoadTfs: {
        "map": entityChooser.currentEntityPath,
        "transforms": [
            {
                "static": true,
                "header": {
                    "frame_id": frameIdInput.currentText,
                    "stamp": { "sec": 0, "nsec": 0 }
                },
                "transform": {
                    "child_frame_id" : childFrameIdInput.currentText,
                    "translation" : {
                        "x" : parseFloat(xInp.text),
                        "y" : parseFloat(yInp.text),
                        "z" : parseFloat(zInp.text)
                    },
                    "rotation" : {
                        "w" : 1.0,
                        "x" : 0.0,
                        "y" : 0.0,
                        "z" : 0.0
                    }
                }
            }
        ]
    }
    Component.onCompleted: {
        primitiveOption.checkCurrentType(appStyle.tmpPrimitiveType)
    }
    Connections {
        target: appStyle
        onTmpPrimitiveTypeChanged: {
            primitiveOption.checkCurrentType(appStyle.tmpPrimitiveType)
        }
        onClickedAction: {
            root.execute()
        }
    }

    onShownChanged: {
        appStyle.tmpUsePreviewMatrix = false
        appStyle.tmpPlacePrimitive = shown
        appStyle.captureMouse = shown
        if(shown) primitiveOption.checkCurrentType(appStyle.tmpPrimitiveType)
    }

    //// UI ////
    RowLayout {
        id: primitiveOption
        property string currentPrimitiveType
        onCurrentPrimitiveTypeChanged: appStyle.tmpPrimitiveType = currentPrimitiveType
        function singleSelect(btn) {
            btn.checked = true
            if(btn !== btn1) btn1.checked = false
            if(btn !== btn2) btn2.checked = false
            if(btn !== btn3) btn3.checked = false
            if(btn !== btn4) btn4.checked = false
            if(btn !== btn5) btn5.checked = false
            if(btn !== btn6) btn6.checked = false
            currentPrimitiveType = getCurrentType()
        }
        function getCurrentType() {
            if(btn1.checked) return "sphere"
            if(btn2.checked) return "plane"
            if(btn3.checked) return "cylinder"
            if(btn4.checked) return "cone"
            if(btn5.checked) return "torus"
            if(btn6.checked) return "cube"
            return ""
        }
        function checkCurrentType( name ) {
            if(name === "sphere") btn1.checked = true
            if(name === "plane") btn2.checked = true
            if(name === "cylinder") btn3.checked = true
            if(name === "cone") btn4.checked = true
            if(name === "torus") btn5.checked = true
            if(name === "cube") btn6.checked = true
        }

        StyledButton {
            id: btn1
            isIcon: true
            checkable: true
            iconSource: "image://primitive/sphere-skinny"
            tooltip: "Place <i>sphere</i>"
            onCheckedChanged: if(checked) primitiveOption.singleSelect(btn1)
        }
        StyledButton {
            id: btn2
            isIcon: true
            checkable: true
            iconSource: "image://primitive/plane-skinny"
            tooltip: "Place <i>plane</i>"
            onCheckedChanged: if(checked) primitiveOption.singleSelect(btn2)
        }
        StyledButton {
            id: btn3
            isIcon: true
            checkable: true
            iconSource: "image://primitive/cylinder-skinny"
            tooltip: "Place <i>cylinder</i>"
            onCheckedChanged: if(checked) primitiveOption.singleSelect(btn3)
        }
        StyledButton {
            id: btn4
            isIcon: true
            checkable: true
            iconSource: "image://primitive/cone-skinny"
            tooltip: "Place <i>cone</i>"
            onCheckedChanged: if(checked) primitiveOption.singleSelect(btn4)
        }
        StyledButton {
            id: btn5
            isIcon: true
            checkable: true
            iconSource: "image://primitive/torus-skinny"
            tooltip: "Place <i>torus</i>"
            onCheckedChanged: if(checked) primitiveOption.singleSelect(btn5)
        }
        StyledButton {
            id: btn6
            isIcon: true
            checkable: true
            iconSource: "image://primitive/cube-skinny"
            tooltip: "Place <i>cube</i>"
            onCheckedChanged: if(checked) primitiveOption.singleSelect(btn6)
        }
        Item {
            Layout.fillWidth: true
        }
        StyledButton {
            id: btnClick
            isIcon: true
            checkable: true
            iconSource: "image://material/ic_location_searching"
            tooltip: "Drag Annotation with mouse directly into the scene"
            onCheckedChanged: {
                appStyle.tmpFollowMouse = checked
            }
            Connections {
                target: appStyle
                onTmpFollowMouseChanged:
                    btnClick.checked = appStyle.tmpFollowMouse
            }
        }
    }
    HelperTarget {
        Layout.fillWidth: true
        id: entityChooser
        currentEntityPath: root.currentEntityPath ? root.currentEntityPath : "/testmap/annotation/annotation_" + (new Date())
        //z: 200
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            Layout.alignment: Qt.AlignTop
            text: "Tr x "
        }
        StyledTextField {
            id: xInp
            Layout.fillWidth: true
            inputMethodHints: Qt.ImhFormattedNumbersOnly
            text: "0"
            onTextChanged: root.updateTranslationMatrix()
        }
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            Layout.alignment: Qt.AlignTop
            text: "Tr y "
        }
        StyledTextField {
            id: yInp
            Layout.fillWidth: true
            inputMethodHints: Qt.ImhFormattedNumbersOnly
            text: "0"
            onTextChanged: root.updateTranslationMatrix()
        }
    }
    RowLayout {
        Layout.fillWidth: true
        StyledLabel {
            Layout.alignment: Qt.AlignTop
            text: "Tr z "
        }
        StyledTextField {
            id: zInp
            Layout.fillWidth: true
            inputMethodHints: Qt.ImhFormattedNumbersOnly
            text: "0"
            onTextChanged: root.updateTranslationMatrix()
        }
    }
    RowLayout {
        Layout.fillWidth: true
        //z:100
        StyledLabel {
            Layout.alignment: Qt.AlignTop
            text: "frame_id:"
        }
        FrameIdChooser {
            id: frameIdInput
            Layout.fillWidth: true
            allowNew: false
        }
    }
    RowLayout {
        Layout.fillWidth: true
        //z:99
        StyledLabel {
            Layout.alignment: Qt.AlignTop
            text: "child_frame_id:"
        }
        FrameIdChooser {
            id: childFrameIdInput
            Layout.fillWidth: true
            allowNew: true
        }
    }
    RowLayout {
        Layout.fillWidth: true
        StyledButton {
            id: executeButton
            Layout.leftMargin: appStyle.controlMargin
            text: "Execute"
            enabled: primitiveOption.getCurrentType() !== "" && entityChooser.valid
            onClicked: {
                root.execute()
            }
        }
    }
}
