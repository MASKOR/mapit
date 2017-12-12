import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1

import ".."
import "../components"

ColumnLayout {
    id: root
    //// in ////
    property bool shown
    property bool editable
    property string currentEntityPath

    function fromParameters(params) {
        entityChooser.currentEntityPath = params.target
        primitiveOption.checkCurrentType(params.type)
    }

    //// out ////
    property bool valid: primitiveOption.getCurrentType() !== "" && entityChooser.valid
    property var parameters: {
        "type": primitiveOption.getCurrentType(),
        "target":entityChooser.currentEntityPath
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
        appStyle.tmpPlacePrimitive = shown
        if(shown) primitiveOption.checkCurrentType(appStyle.tmpPrimitiveType)
    }
    //// UI ////
    RowLayout {
        Layout.fillWidth: true
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
        Layout.fillWidth: true
        id: entityChooser
        currentEntityPath: root.currentEntityPath
    }
}
