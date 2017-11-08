//TODO: Rename this file
import QtQuick 2.7
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
    property var currentOperator
    onCurrentOperatorChanged: opPane.loadOperatorDialog(currentOperator)
    property var currentPipeline
    onCurrentPipelineChanged: opPane.loadPipelineDialog(currentPipeline)
    property alias currentCheckout: opPane.currentCheckout
    property alias currentEntityPath: opPane.currentEntityPath
    property string currentFrameId
    function formatVec3(vec) {
        return "(" + vec.x.toFixed(2) + ", " + vec.y.toFixed(2) + ", " + vec.z.toFixed(2) + ")"
    }

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
            id: entityInfo
            Layout.fillWidth: true
            Layout.margins: appStyle.controlMargin
            visible: entityHeader.checked && root.currentEntityPath !== undefined
            property var currentEntity: root.currentEntityPath ? currentCheckout.getEntity(root.currentEntityPath) : null
            property string currentEntityType: currentEntity ? currentEntity.type : ""
            property string currentEntityFrameId: currentEntity ? currentEntity.frameId : ""
            property string currentEntityStamp: currentEntity ? currentEntity.stamp : ""
            property var currentEntitydata: root.currentEntityPath ? currentCheckout.getEntitydataReadOnly(root.currentEntityPath) : null
            property bool isPointcloud: entityInfo.currentEntityType === "layertype_pointcloud2" || entityInfo.currentEntityType === "layertype_las"
            GridLayout {
                Layout.fillWidth: true
                columns: 2
                StyledLabel {
                    text: "Type:"
                    font.weight: Font.Bold
                }
                StyledLabel {
                    text: entityInfo.currentEntityType
                }
                StyledLabel {
                    text: "frame_id:"
                    font.weight: Font.Bold
                }
                StyledLabel {
                    text: entityInfo.currentEntityFrameId
                }
                StyledLabel {
                    text: "Timestamp:"
                    font.weight: Font.Bold
                }
                StyledLabel {
                    text: entityInfo.currentEntityStamp
                }
                StyledLabel {
                    text: "Fields:"
                    font.weight: Font.Bold
                }
                Flow {
                    id: infoFlow
                    visible: entityInfo.isPointcloud
                    Layout.fillWidth: true
                    Repeater {
                        id: repeater
                        onItemAdded: infoFlow.forceLayout()
                        model: entityInfo.currentEntitydata.getInfo("fields")
                        StyledLabel {
                            text: modelData.name + ((index != (repeater.count-1))?", ":"")
                        }
                    }
                }
                StyledLabel {
                    visible: entityInfo.isPointcloud
                    text: "# Pts.:"
                    font.weight: Font.Bold
                }
                StyledLabel {
                    visible: entityInfo.isPointcloud
                    text: entityInfo.currentEntitydata.getInfo("width")
                }
                StyledLabel {
                    visible: entityInfo.isPointcloud
                    text: "Min:"
                    font.weight: Font.Bold
                }
                StyledLabel {
                    visible: entityInfo.isPointcloud
                    text: formatVec3(entityInfo.currentEntitydata.getInfo("min"))
                }
                StyledLabel {
                    visible: entityInfo.isPointcloud
                    text: "Max:"
                    font.weight: Font.Bold
                }
                StyledLabel {
                    visible: entityInfo.isPointcloud
                    text: formatVec3(entityInfo.currentEntitydata.getInfo("max"))
                }
            }
            UPNS.TfTransform {
                id: currentEntitydataTransform
                path: root.currentEntityPath
                checkout: root.currentCheckout
                targetFrame: root.currentFrameId
                sourceFrame:  root.currentCheckout.getEntity(currentEntityPath).frameId
                mustExist: false
                onMatrixChanged: {
                    var m = matrix
                    var m0 = m.row(0)
                    var m1 = m.row(1)
                    var m2 = m.row(2)
                    var m3 = m.row(3)
                    m00.text = m0.x.toFixed(2); m01.text = m0.y.toFixed(2); m02.text = m0.z.toFixed(2); m03.text = m0.w.toFixed(2)
                    m10.text = m1.x.toFixed(2); m11.text = m1.y.toFixed(2); m12.text = m1.z.toFixed(2); m13.text = m1.w.toFixed(2)
                    m20.text = m2.x.toFixed(2); m21.text = m2.y.toFixed(2); m22.text = m2.z.toFixed(2); m23.text = m2.w.toFixed(2)
                    m30.text = m3.x.toFixed(2); m31.text = m3.y.toFixed(2); m32.text = m3.z.toFixed(2); m33.text = m3.w.toFixed(2)
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
