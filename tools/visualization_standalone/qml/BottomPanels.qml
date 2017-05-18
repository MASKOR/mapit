import QtQuick 2.4
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtQuick.Window 2.2 as Wnd
import QtGraphicalEffects 1.0
import pcl 1.0
import fhac.upns 1.0 as UPNS
import QtQuick 2.0 as QQ2
import "panes"

Item {
    id: root
    property alias currentOperator: opPane.currentOperator
    property alias currentCheckout: opPane.currentCheckout
    property alias currentEntityPath: opPane.currentEntityPath
    ColumnLayout {
        anchors.fill: parent
        StyledHeader {
            Layout.fillWidth: true
            id: operatorHeader
            text: qsTr("Operator")
        }
        OperatorPane {
            id: opPane
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
        StyledHeader {
            Layout.fillWidth: true
            id: entityHeader
            text: qsTr("Entity")
        }
        ColumnLayout {
            Layout.fillWidth: true
            visible: entityHeader.checked && root.currentEntityPath !== undefined
            RowLayout {
                Layout.fillWidth: true
                StyledLabel {
                    Layout.fillWidth: true
                    text: "Type: "
                    font.weight: Font.Bold
                }

                StyledLabel {
                    Layout.fillWidth: true
                    text: root.currentEntityPath ? currentCheckout.getEntity(root.currentEntityPath).type : ""
                }
            }
            UPNS.EntitydataTransform {
                id: currentEntitydataTransform
                path: root.currentEntityPath + ".tf"
                checkout: root.currentCheckout
                mustExist: false
                onMatrixChanged: {
                    var m = matrix
                    var m0 = m.row(0)
                    var m1 = m.row(1)
                    var m2 = m.row(2)
                    var m3 = m.row(3)
                    m00.text = m0.x; m01.text = m0.y; m02.text = m0.z; m03.text = m0.w
                    m10.text = m1.x; m11.text = m1.y; m12.text = m1.z; m13.text = m1.w
                    m20.text = m2.x; m21.text = m2.y; m22.text = m2.z; m23.text = m2.w
                    m30.text = m3.x; m31.text = m3.y; m32.text = m3.z; m33.text = m3.w
                    console.log("Executed: Pos:" + m30.text + " " + m31.text + " " + m33.text )
                }
            }
            RowLayout {
                visible: currentEntitydataTransform.exists
                Layout.fillWidth: true
                StyledLabel {
                    Layout.fillWidth: true
                    text: "Transform: "
                    font.weight: Font.Bold
                }

                GridLayout {
                    StyledLabel {
                        id:m00
                        Layout.column: 0
                        Layout.row: 0
                    }
                    StyledLabel {
                        id:m01
                        Layout.column: 1
                        Layout.row: 0
                    }
                    StyledLabel {
                        id:m02
                        Layout.column: 2
                        Layout.row: 0
                    }
                    StyledLabel {
                        id:m03
                        Layout.column: 3
                        Layout.row: 0
                    }

                    StyledLabel {
                        id:m10
                        Layout.column: 0
                        Layout.row: 1
                    }
                    StyledLabel {
                        id:m11
                        Layout.column: 1
                        Layout.row: 1
                    }
                    StyledLabel {
                        id:m12
                        Layout.column: 2
                        Layout.row: 1
                    }
                    StyledLabel {
                        id:m13
                        Layout.column: 3
                        Layout.row: 1
                    }

                    StyledLabel {
                        id:m20
                        Layout.column: 0
                        Layout.row: 2
                    }
                    StyledLabel {
                        id:m21
                        Layout.column: 1
                        Layout.row: 2
                    }
                    StyledLabel {
                        id:m22
                        Layout.column: 2
                        Layout.row: 2
                    }
                    StyledLabel {
                        id:m23
                        Layout.column: 3
                        Layout.row: 2
                    }

                    StyledLabel {
                        id:m30
                        Layout.column: 0
                        Layout.row: 3
                    }
                    StyledLabel {
                        id:m31
                        Layout.column: 1
                        Layout.row: 3
                    }
                    StyledLabel {
                        id:m32
                        Layout.column: 2
                        Layout.row: 3
                    }
                    StyledLabel {
                        id:m33
                        Layout.column: 3
                        Layout.row: 3
                    }
                }
            }
        }
    }
}
