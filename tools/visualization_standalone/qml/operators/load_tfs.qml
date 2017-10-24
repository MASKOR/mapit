import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

import fhac.upns 1.0 as UPNS

import ".."

Item {
    id: root
    //// in ////
    property bool shown
    property bool editable
    property var currentCheckout
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
    property bool valid: currentEntitydataTransformId.path != ""
    property var parameters: {
        "map": entityChooser.currentEntityPath,
        "transforms": [
            {   "static": true,
                "header": {
                    "frame_id": todo.frame_id_name,
                    "stamp": { "sec": 0, "nsec": 0 }
                },
                "transform": {
                    "child_frame_id" : todo.child_frame_id,
                    "translation" : {
                        "x" : translationMatrix.m00,
                        "y" : translationMatrix.m01,
                        "z" : translationMatrix.m02
                    },
                    "rotation" : {
                        "w" : rangle,
                        "x" : rx,
                        "y" : ry,
                        "z" : rz
                    }
                }
            }
            ],
        "frame_id": frameIdChooser.currentFrameId,
        "child_frame_id": {
            "mat": finalMatrixArray
        }
    }
    UPNS.EntitydataTransform {
        id: currentEntitydataTransformId
        checkout: currentCheckout
        mustExist: false
        path: entityChooser.currentEntityPath + ((entityChooser.currentEntityPath.length > 3
                                      && entityChooser.currentEntityPath.lastIndexOf(".tf") !== entityChooser.currentEntityPath.length-3)
                                        ? ".tf" : "")
    }
    Component.onCompleted: {
        appStyle.tmpPreviewMatrix = finalMatrix
        appStyle.tmpUsePreviewMatrix = true
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

    property matrix4x4 finalMatrix: matRotX.times(matRotY.times(matRotZ)).times(scaleInp.text).times(translationMatrix);
    property matrix4x4 translationMatrix

    property real deg2rad: appStyle.useRadians ? 1 : Math.PI/180.0
    property real rx: rxInp.text*deg2rad
    property real ry: ryInp.text*deg2rad
    property real rz: rzInp.text*deg2rad
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
        translateMatrix.translate(xInp.text, yInp.text, zInp.text)
        root.translationMatrix = translateMatrix
    }

    //// UI ////
    ColumnLayout {
        anchors.fill: parent
        HelperTarget {
            id: entityChooser
            currentEntityPath: root.currentEntityPath
            dialogRoot: root
            text: "Map: "
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
        Item {
            Layout.fillHeight: true
        }
        SystemPalette {
            id: palette
        }
    }
}
