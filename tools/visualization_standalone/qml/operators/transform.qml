import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

import fhac.upns 1.0 as UPNS

import ".."

Item {
    id: root
    //// in ////
    property bool editable
    property var currentCheckout
    property string currentEntityPath

    readonly property string angleUnit: appStyle.useRadians ? "rad" : qsTr( "\u00B0" ) // degree
    readonly property string angleUnitBracket: "(" + angleUnit + ")"
    function fromParameters(params) {
        entityChooser.currentEntityPath = params.target
        scaleInp.text = params.tf.mat[0];
    }

    //// out ////
    property bool valid: currentEntitydataTransformId.path != ""
    property var parameters: {
        "target": currentEntitydataTransformId.path,
        "mode": "absolute",
        "tf": {
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

    property matrix4x4 finalMatrix: matRotX.times(matRotY.times(matRotZ)).times(scaleInp.text);

    property real deg2rad: appStyle.useRadians ? 1 : Math.PI/180.0
    property real rx: rxInp.text*deg2rad
    property real ry: ryInp.text*deg2rad
    property real rz: rzInp.text*deg2rad
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

    //// UI ////
    ColumnLayout {
        anchors.fill: parent
        HelperTarget {
            id: entityChooser
            currentEntityPath: root.currentEntityPath
            dialogRoot: root
        }
        RowLayout {
            Layout.fillWidth: true
            Text {
                Layout.alignment: Qt.AlignTop
                text: "Scale:"
                color: palette.text
                renderType: Text.NativeRendering
            }
            TextField {
                id: scaleInp
                Layout.fillWidth: true
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                text: "1"
            }
        }
        RowLayout {
            Layout.fillWidth: true
            Text {
                Layout.alignment: Qt.AlignTop
                text: "Rot x " + root.angleUnitBracket
                color: palette.text
                renderType: Text.NativeRendering
            }
            TextField {
                id: rxInp
                Layout.fillWidth: true
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                text: "-90"
            }
        }
        RowLayout {
            Layout.fillWidth: true
            Text {
                Layout.alignment: Qt.AlignTop
                text: "Rot y " + root.angleUnitBracket
                color: palette.text
                renderType: Text.NativeRendering
            }
            TextField {
                id: ryInp
                Layout.fillWidth: true
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                text: "0"
            }
        }
        RowLayout {
            Layout.fillWidth: true
            Text {
                Layout.alignment: Qt.AlignTop
                text: "Rot z " + root.angleUnitBracket
                color: palette.text
                renderType: Text.NativeRendering
            }
            TextField {
                id: rzInp
                Layout.fillWidth: true
                inputMethodHints: Qt.ImhFormattedNumbersOnly
                text: "0"
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
