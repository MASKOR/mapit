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
import Qt3D.Core 2.0

import fhac.mapit 1.0 as Mapit

import ".."
import "../components"

ColumnLayout {
    id: root
    //// in ////
    property bool shown
    property bool editable
    property string currentWorkspace
    property string currentEntityPath

    readonly property string angleUnit: appStyle.useRadians ? "rad" : qsTr( "\u00B0" ) // degree
    readonly property string angleUnitBracket: "(" + angleUnit + ")"
    function fromParameters(params) {
        entityChooser.currentEntityPath = params.target
        scaleInp.text = params.tf.mat[0];
        xInp.text = params.tf.mat[12];
        yInp.text = params.tf.mat[13];
        zInp.text = params.tf.mat[14];
    }

    //// out ////
    property bool valid: true
    property var parameters: {
        "prefix": entityChooser.currentEntityPath,
        "transforms": [
            {   "static": isStatic.checked,
                "header": {
                    "frame_id": frameIdInput.text,
                    "stamp": {
                                "sec": parseInt(secInput.text),
                                "nsec": parseInt(nsecInput.text)
                            }
                },
                "transform": {
                    "child_frame_id" : childFrameIdInput.text,
                    "translation" : {
                        //"x" : finalMatrixArray[3],
                        //"y" : finalMatrixArray[7],
                        //"z" : finalMatrixArray[11]
                        "x" : parseFloat(xInp.text),
                        "y" : parseFloat(yInp.text),
                        "z" : parseFloat(zInp.text)
                    },
                    "rotation" : {
                        "w" : dummyTransform3d.quaternionRot.scalar,
                        "x" : dummyTransform3d.quaternionRot.x,
                        "y" : dummyTransform3d.quaternionRot.y,
                        "z" : dummyTransform3d.quaternionRot.z
                    }
                }
            }
            ],
        "frame_id": frameIdInput.currentFrameId
    }
    Item {
        id: priv
        Mapit.TfTransform {
            id: currentEntitydataTransformId
            workspace: globalApplicationState.currentWorkspace
            mustExist: false
            path: entityChooser.currentEntityPath /*+ ((entityChooser.currentEntityPath.length > 3
                                          && entityChooser.currentEntityPath.lastIndexOf(".tf") !== entityChooser.currentEntityPath.length-3)
                                            ? ".tf" : "")*/
        }
        Transform {
            id: dummyTransform3d
            property vector3d rotationAxisInput: Qt.vector3d(rx,ry,rz)
            property vector3d rotationAxis: rotationAxisInput.normalized()
            property quaternion quaternionRot: fromAxisAndAngle(rotationAxis, rangle)
            property matrix4x4 rotationMatrix: rotateAround(Qt.vector3d(0,0,0), rangle, Qt.vector3d(rx,ry,rz))
            rotation: quaternionRot
        }
    }

    Component.onCompleted: {
        appStyle.tmpPreviewMatrix = finalMatrix
        appStyle.tmpUsePreviewMatrix = true
        appStyle.tmpPlacePrimitive = false
    }
    Component.onDestruction: appStyle.tmpUsePreviewMatrix = false
    onVisibleChanged: appStyle.tmpUsePreviewMatrix = visible
    onShownChanged: appStyle.tmpUsePreviewMatrix = shown

    property var m0: finalMatrix.row(0)
    property var m1: finalMatrix.row(1)
    property var m2: finalMatrix.row(2)
    property var m3: finalMatrix.row(3)
    property var finalMatrixArray: [m0.x, m0.y, m0.z, m0.w
                                   ,m1.x, m1.y, m1.z, m1.w
                                   ,m2.x, m2.y, m2.z, m2.w
                                   ,m3.x, m3.y, m3.z, m3.w ]

    onFinalMatrixChanged: {
        appStyle.tmpPreviewMatrix = finalMatrix
    }

    //property matrix4x4 rotationMatrix: matRotX.times(matRotY.times(matRotZ))
    property matrix4x4 rotationMatrix: dummyTransform3d.matrix//Qt.quaternion(rangle, rx,ry,rz).
    property matrix4x4 finalMatrix: rotationMatrix.times(translationMatrix.times(scaleInp.text));
    property matrix4x4 translationMatrix

    property real deg2rad: appStyle.useRadians ? Math.PI/180.0 : 1
    property real rx: rxInp.text//*deg2rad
    property real ry: ryInp.text//*deg2rad
    property real rz: rzInp.text//*deg2rad
    property real rangle: rangleInp.text*deg2rad
    //TODO: replace with rotate/...
    property matrix4x4 matRotX: Qt.matrix4x4(1, 0, 0, 0,
                                             0,  Math.cos(rx), -Math.sin(rx), 0,
                                             0,  Math.sin(rx),  Math.cos(rx), 0,
                                             0, 0, 0, 1);
    property matrix4x4 matRotY: Qt.matrix4x4( Math.cos(ry), 0, Math.sin(ry), 0,
                                             0, 1, 0, 0,
                                             -Math.sin(ry), 0, Math.cos(ry), 0,
                                             0, 0, 0, 1);
    property matrix4x4 matRotZ: Qt.matrix4x4(Math.cos(rz), -Math.sin(rz), 0, 0,
                                             Math.sin(rz),  Math.cos(rz), 0, 0,
                                             0, 0, 1, 0,
                                             0, 0, 0, 1);

    function updateTranslationMatrix() {
        var translateMatrix = Qt.matrix4x4()
        if(xInp == null || yInp == null || zInp == null) return // Note: the line below produced error logs. bug?
        translateMatrix.translate(xInp.text, yInp.text, zInp.text)
        root.translationMatrix = translateMatrix
    }

    //// UI ////
    ColumnLayout {
        Layout.fillWidth: true
        HelperTarget {
            id: entityChooser
            currentEntityPath: root.currentEntityPath
            text: "Prefix: "
        }
        RowLayout {
            Layout.fillWidth: true
            StyledLabel {
                Layout.alignment: Qt.AlignTop
                text: "frame_id:"
            }
            StyledTextField {
                id: frameIdInput
                Layout.fillWidth: true
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                text: "testframeid"
                onTextChanged: console.log(JSON.stringify(root.parameters))
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
            StyledLabel {
                enabled: false
                Layout.alignment: Qt.AlignTop
                text: "Scale:"
            }
            StyledTextField {
                enabled: false
                id: scaleInp
                Layout.fillWidth: true
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                text: "1"
            }
        }
        RowLayout {
            Layout.fillWidth: true
            StyledLabel {
                Layout.alignment: Qt.AlignTop
                text: "Rot x " //+ root.angleUnitBracket
            }
            StyledTextField {
                id: rxInp
                Layout.fillWidth: true
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                text: "0"
            }
        }
        RowLayout {
            Layout.fillWidth: true
            StyledLabel {
                Layout.alignment: Qt.AlignTop
                text: "Rot y " //+ root.angleUnitBracket
            }
            StyledTextField {
                id: ryInp
                Layout.fillWidth: true
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                text: "0"
            }
        }
        RowLayout {
            Layout.fillWidth: true
            StyledLabel {
                Layout.alignment: Qt.AlignTop
                text: "Rot z " //+ root.angleUnitBracket
            }
            StyledTextField {
                id: rzInp
                Layout.fillWidth: true
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                text: "0"
            }
        }

        RowLayout {
            Layout.fillWidth: true
            StyledLabel {
                Layout.alignment: Qt.AlignTop
                text: "Angle " + root.angleUnitBracket
            }
            StyledTextField {
                id: rangleInp
                Layout.fillWidth: true
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                text: "0"
            }
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
            StyledLabel {
                text: "Seconds: "
            }
            StyledTextField {
                id: secInput
                Layout.fillWidth: true
                validator: IntValidator {}
                text: "0"
            }
        }
        RowLayout {
            Layout.fillWidth: true
            StyledLabel {
                text: "Nanoseconds: "
            }
            StyledTextField {
                id: nsecInput
                Layout.fillWidth: true
                validator: IntValidator {}
                text: "0"
            }
        }
        SystemPalette {
            id: palette
        }
        RowLayout {
            Layout.fillWidth: true
            StyledLabel {
                text: "Is Static:"
            }
            StyledCheckBox {
                id: isStatic
            }
        }
    }
}
