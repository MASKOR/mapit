import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1

import ".."
import "../operators"

Item {
    id: root
    height: implicitHeight

    //// in ////
    property bool shown
    property bool editable
    property var currentCheckout
    property string currentEntityPath

    function fromParameters(params) {
        entityChooser.currentEntityPath = params.target
        primitiveOption.checkCurrentType(params.type)
    }
    function updateTranslationMatrix() {
        var translateMatrix = Qt.matrix4x4()
        translateMatrix.translate(xInp.text, yInp.text, zInp.text)
        appStyle.tmpPreviewMatrix = translateMatrix
    }

    //// out ////
    property bool valid: primitiveOption.getCurrentType() !== "" && entityChooser.valid
    property var parameters: {
        "type": primitiveOption.getCurrentType(),
        "target": entityChooser.currentEntityPath
    }
    property var parametersLoadTfs: {
        "map": entityChooser.currentEntityPath,
        "transforms": [
            {
                "static": true,
                "header": {
                    "frame_id": frameIdInput.currentText + "_annotation_" + entityChooser.currentEntityPath,
                    "stamp": { "sec": 0, "nsec": 0 }
                },
                "transform": {
                    "child_frame_id" : childFrameIdInput.text,
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
        ],
        "frame_id": frameIdInput.currentText
    }
    Component.onCompleted: {
        primitiveOption.checkCurrentType(appStyle.tmpPrimitiveType)
    }
    Connections {
        target: appStyle
        onTmpPrimitiveTypeChanged: {
            primitiveOption.checkCurrentType(appStyle.tmpPrimitiveType)
        }
    }

    onShownChanged: {
        appStyle.tmpUsePreviewMatrix = shown
        appStyle.tmpPlacePrimitive = shown
        if(shown) primitiveOption.checkCurrentType(appStyle.tmpPrimitiveType)
    }

    //// UI ////
    ColumnLayout {
        anchors.fill: parent
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
                id:btn1
                isIcon: true
                checkable: true
                iconSource: "image://primitive/sphere-skinny"
                onCheckedChanged: if(checked) primitiveOption.singleSelect(btn1)
            }
            StyledButton {
                id:btn2
                isIcon: true
                checkable: true
                iconSource: "image://primitive/plane-skinny"
                onCheckedChanged: if(checked) primitiveOption.singleSelect(btn2)
            }
            StyledButton {
                id:btn3
                isIcon: true
                checkable: true
                iconSource: "image://primitive/cylinder-skinny"
                onCheckedChanged: if(checked) primitiveOption.singleSelect(btn3)
            }
            StyledButton {
                id:btn4
                isIcon: true
                checkable: true
                iconSource: "image://primitive/cone-skinny"
                onCheckedChanged: if(checked) primitiveOption.singleSelect(btn4)
            }
            StyledButton {
                id:btn5
                isIcon: true
                checkable: true
                iconSource: "image://primitive/torus-skinny"
                onCheckedChanged: if(checked) primitiveOption.singleSelect(btn5)
            }
            StyledButton {
                id:btn6
                isIcon: true
                checkable: true
                iconSource: "image://primitive/cube-skinny"
                onCheckedChanged: if(checked) primitiveOption.singleSelect(btn6)
            }
        }
        HelperTarget {
            id: entityChooser
            currentEntityPath: root.currentEntityPath
            dialogRoot: root
            z: 200
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
            z:100
            StyledLabel {
                Layout.alignment: Qt.AlignTop
                text: "frame_id:"
            }
            QuickAccessMenu {
                z:100
                id: frameIdInput
                Layout.fillWidth: true
                height: appStyle.controlHeightInner
                allowNew: true
                model: currentCheckout.getFrameIds()
                blurMouseArea: MouseArea {
                    parent: root.parent.parent
                    anchors.fill: parent
                    preventStealing: true
                    propagateComposedEvents: true
                    z:-1000
                }
            }
        }
        RowLayout {
            Layout.fillWidth: true
            StyledLabel {
                Layout.alignment: Qt.AlignTop
                text: "child_frame_id:"
            }
            StyledTextField {
                id: childFrameIdInput
                Layout.fillWidth: true
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                text: "entity_frame_id"
            }
        }
        RowLayout {
            Layout.fillWidth: true
            StyledButton {
                Layout.leftMargin: appStyle.controlMargin
                text: "Execute"
                enabled: primitiveOption.getCurrentType() !== "" && entityChooser.valid
                onClicked: {
                    currentCheckout.doOperation(currentOperator.moduleName, currentOperatorUiItem.parameters)
                }
            }
        }
        Item {
            Layout.fillHeight: true
        }
    }
}
