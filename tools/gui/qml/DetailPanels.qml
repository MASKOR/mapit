/*******************************************************************************
 *
 * Copyright 2017-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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

import QtQuick 2.7
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtQuick.Window 2.2 as Wnd
import QtGraphicalEffects 1.0
import pcl 1.0
import QtQuick 2.0 as QQ2
import "panes"
import "components"
Item {
    id: root
    function formatVec3(vec) {
        if(!vec) return "(NaN,NaN,NaN,NaN,Watman)"
        return "(" + vec.x.toFixed(2) + ", " + vec.y.toFixed(2) + ", " + vec.z.toFixed(2) + ")"
    }

    Item {
        id: popupLayerRight
        anchors.fill: parent
        z: 1000
        Component.onCompleted: {
            appStyle.popupLayerRight = popupLayerRight
        }
    }

    Connections {
        // Connect all content to the global application state
        target: globalApplicationState
        onCurrentDetailDialogChanged: opPane.detailDialogPath = globalApplicationState.currentDetailDialog
    }
    Connections {
        // show info about current transform in detail pane
        target: globalApplicationState.currentEntityTransform
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

    QCtl.BusyIndicator {
        z: 100
        property var pos: entityInfo.mapToItem(parent, (entityInfo.width-width)*0.5, (entityInfo.height-height)*0.5)
        x: pos.x
        y: pos.y
        running: entityInfo.loading
    }
    ColumnLayout {
        id: detailTab
        anchors.fill: parent
        StyledHeader {
            Layout.fillWidth: true
            id: operatorHeader
            text: qsTr("Operator")
            iconSource: "image://material/ic_input"
        }
        OperatorPane {
            id: opPane
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentWorkspace: globalApplicationState.currentWorkspace
        }
        StyledHeader {
            Layout.fillWidth: true
            id: entityHeader
            text: qsTr("Entity")
            iconSource: "image://material/ic_poll"
        }
        ColumnLayout {
            id: entityInfo
            Layout.fillWidth: true
            Layout.margins: appStyle.controlMargin
            visible: entityHeader.checked && currentEntity !== undefined
            property var currentEntity: globalApplicationState.currentEntity
            property string currentEntityType: currentEntity && currentEntity.isValid() ? currentEntity.type : ""
            property string currentEntityFrameId: currentEntity && currentEntity.isValid() ? currentEntity.frameId : ""
            property string currentEntityStamp: currentEntity && currentEntity.isValid() ? currentEntity.stamp.text : ""
            property var currentEntitydata: globalApplicationState.currentEntitydata
            property bool loading: currentEntitydata.isLoading
            property bool isPointcloud: entityInfo.currentEntityType === "layertype_pointcloud2" || entityInfo.currentEntityType === "layertype_las"
            property bool isOpenVDB: entityInfo.currentEntityType === "layertype_openvdb"
            property bool isPrimitive: entityInfo.currentEntityType === "layertype_primitive"

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
                    visible: entityInfo.isPrimitive
                    text: "Type:"
                    font.weight: Font.Bold
                }
                StyledLabel {
                    visible: entityInfo.isPrimitive && !entityInfo.loading

                    text: visible && entityInfo.currentEntitydata ? entityInfo.currentEntitydata.info["type"] ? entityInfo.currentEntitydata.info["type"] : "" : ""
                }
                StyledLabel {
                    visible: entityInfo.isPrimitive
                    text: "Text:"
                    font.weight: Font.Bold
                }
                StyledLabel {
                    visible: entityInfo.isPrimitive && !entityInfo.loading
                    text: visible && entityInfo.currentEntitydata ? entityInfo.currentEntitydata.info["text"] ? entityInfo.currentEntitydata.info["text"] : "" : ""
                }
                StyledLabel {
                    visible: entityInfo.isPointcloud
                    text: "Fields:"
                    font.weight: Font.Bold
                }
                Flow {
                    id: infoFlow
                    visible: entityInfo.isPointcloud && !entityInfo.loading
                    Layout.fillWidth: true
                    Repeater {
                        id: repeater
                        onItemAdded: infoFlow.forceLayout()
                        model: entityInfo.currentEntitydata ? entityInfo.currentEntitydata.info["fields"] : 0
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
                    visible: entityInfo.isPointcloud && !entityInfo.loading
                    property var infoWidth: visible && entityInfo.currentEntitydata ? entityInfo.currentEntitydata.info["width"] : ""
                    text: (typeof infoWidth) !== "undefined" ? infoWidth : ""
                }
                StyledLabel {
                    visible: entityInfo.isPointcloud || entityInfo.isOpenVDB || entityInfo.isPrimitive
                    text: "Min:"
                    font.weight: Font.Bold
                }
                StyledLabel {
                    visible: (entityInfo.isPointcloud || entityInfo.isOpenVDB || entityInfo.isPrimitive) && !entityInfo.loading
                    text: formatVec3(entityInfo.currentEntitydata ? entityInfo.currentEntitydata.info["min"] : Qt.vector3d(0,0,0))
                }
                StyledLabel {
                    visible: entityInfo.isPointcloud || entityInfo.isOpenVDB || entityInfo.isPrimitive
                    text: "Max:"
                    font.weight: Font.Bold
                }
                StyledLabel {
                    visible: (entityInfo.isPointcloud || entityInfo.isOpenVDB || entityInfo.isPrimitive) && !entityInfo.loading
                    text: formatVec3(entityInfo.currentEntitydata ? entityInfo.currentEntitydata.info["max"] : Qt.vector3d(0,0,0))
                }
            }
            RowLayout {
                visible: globalApplicationState.currentEntityTransform.exists
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
