import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1

import fhac.upns 1.0
//import "components"
//import "operators"

ApplicationWindow {
    title: qsTr("Map Visualization")
    width: 640
    height: 480
    visible: true
    menuBar: MenuBar {
        Menu {
            title: qsTr("&File")
            MenuItem {
                text: qsTr("&Open Project")
                onTriggered: openFileDialog.open()
            }
            MenuItem {
                text: qsTr("&Commit")
                onTriggered: saveFileDialog.open()
            }
            MenuItem {
                text: qsTr("Save Project &As")
                onTriggered: saveFileDialog.open()
            }
            MenuItem {
                text: qsTr("E&xit")
                onTriggered: Qt.quit();
            }
        }
        Menu {
            title: qsTr("&Edit")
            MenuItem {
                text: qsTr("&Copy")
                onTriggered: console.log("not yet implemented.");
            }
            MenuItem {
                text: qsTr("&Paste")
                onTriggered: {

                }
            }
        }
        Menu {
            title: qsTr("&View")
//            MenuItem {
//                id: detailHiItem
//                text: qsTr("&Hi Detail")
//                checkable: true
//                checked: true
//                onCheckedChanged: {
//                }
//            }
//            MenuItem {
//                id: detailMidtem
//                text: qsTr("&Mid Detail")
//                checkable: true
//                checked: false
//                onCheckedChanged: {
//                }
//            }
//            MenuItem {
//                id: detailLoItem
//                text: qsTr("&Low Detail")
//                checkable: true
//                checked: false
//                onCheckedChanged: {
//                }
//            }
            MenuSeparator { }
//            MenuItem {
//                id: executeItem
//                text: qsTr("Lower Detail when &moving")
//                enabled: false
//                onTriggered: {
//                }
//            }
            MenuSeparator { }
            MenuItem {
                id: showCenter
                text: qsTr("Show Center &Cross")
                checkable: true
                checked: true
            }
            MenuSeparator { }
//            MenuItem {
//                id: centerFixed
//                text: qsTr("Torso Yaw fixed to view")
//                checkable: true
//                checked: true
//            }
            MenuItem {
                id: fixedUpvec
                text: qsTr("Fixed Upvector")
                checkable: true
                checked: true
            }
            MenuItem {
                id: invertY
                text: qsTr("Invert Y Axis")
                enabled: !fixedUpvec.checked
                checkable: true
                checked: true
            }
            MenuSeparator { }
            MenuItem {
                id: vrModeEnabled
                text: qsTr("Enable VR")
                checkable: true
                checked: true
            }
            Menu {
                id: vrMirror
                title: qsTr("Mirror VR")
                enabled: vrModeEnabled.checked
                ExclusiveGroup {
                    id: mirrorGroup
                }
                MenuItem {
                    id: vrMirrorOff
                    text: qsTr("No Mirror")
                    checkable: drawingArea.running
                    checked: false
                    exclusiveGroup: mirrorGroup
                }
                MenuItem {
                    id: vrMirrorDistorsion
                    text: qsTr("Distorsion")
                    checkable: drawingArea.running
                    checked: true
                    exclusiveGroup: mirrorGroup
                }
                MenuItem {
                    id: vrMirrorRight
                    text: qsTr("Right Eye")
                    enabled: false
                    checkable: false && drawingArea.running // not yet available
                    checked: false
                    exclusiveGroup: mirrorGroup
                }
                MenuItem {
                    id: vrMirrorLeft
                    text: qsTr("Left Eye")
                    enabled: false
                    checkable: false && drawingArea.running // not yet available
                    checked: false
                    exclusiveGroup: mirrorGroup
                }
            }
        }
        Menu {
            title: qsTr("&Window")
            MenuItem {
                text: qsTr("Show &Operations Pane")
                checkable: true
                checked: true
                onCheckedChanged: {
                }
            }
            MenuItem {
                text: qsTr("Show &Maps/Layers Pane")
                checkable: true
                checked: false
                onCheckedChanged: {
                }
            }
        }
    }
    GridLayout {
        anchors.fill: parent
        MapsRenderViewport {
            id: drawingArea
            Layout.minimumWidth: 50
            Layout.fillWidth: true
            Layout.fillHeight: true
//            entitydata: EntityDataPointcloud2 {
//                //filename: "data/bunny.pcd"
//                //filename: "data/alignedRGB_FH/Aligned_FARO_Scan_072.ply"
//                //filename: "data/fh/000000.pcd"

//                //filename: "data/fh/all_pointclouds20_mini_norm.pcd"
//                filename: "data/fh/all_pointclouds20_norm_flipped.pcd"
//            }
            filename: "data/fh/all_pointclouds20_norm_flipped.pcd"

            //filename: "data/Rover.pcd"
            vrmode: vrModeEnabled.checked
            mirrorEnabled: !vrMirrorOff.checked
            mirrorDistorsion: vrMirrorDistorsion.checked
            mirrorRightEye: vrMirrorRight.checked

            onFrame: {
                xboxController.onFrame()
            }
            Rectangle {
                x: parent.width/2
                y: parent.height/2-5
                width: 1
                height: 11
                color: "white"
                visible: showCenter.checked
            }
            Rectangle {
                x: parent.width/2-5
                y: parent.height/2
                width: 11
                height: 1
                color: "white"
                visible: showCenter.checked
            }
            Text {
                color: "white"
                id: name
                text: "Pos: (" + drawingArea.torsoPos.x + ", " + drawingArea.torsoPos.y + ", " + drawingArea.torsoPos.z + ")"
            }

            property bool moveAlongHeadDirection: true // move where head shows or where torso shows?
            property alias fixedUpVector: fixedUpvec.checked

            property var torsoPos: Qt.vector3d(0.0,0.0,0.0)
            property var torsoPositionMatrix: Qt.matrix4x4(
                                     1.0, 0.0, 0.0, torsoPos.x,
                                     0.0, 1.0, 0.0, torsoPos.y,
                                     0.0, 0.0, 1.0, torsoPos.z,
                                     0.0, 0.0, 0.0, 1.0)
            property var torsoOrientation: Qt.matrix4x4(
                                               1.0, 0.0, 0.0, 0.0,
                                               0.0, 1.0, 0.0, 0.0,
                                               0.0, 0.0, 1.0, 0.0,
                                               0.0, 0.0, 0.0, 1.0)


            // inverse head matrix
            property var headOrientationInverse: headOrientation.transposed()

            // orientation to yaw and pitch around (always view)
            property var rotationOrientation: headOrientationInverse.times(torsoOrientation)

            // orientation to move along (may be torso or head)
            property var moveOrientation: moveAlongHeadDirection?rotationOrientation:torsoOrientation

            // output
            matrix: torsoOrientation.times(torsoPositionMatrix);

            function move(side, up, forward) {
                torsoPos = torsoPos.plus(moveOrientation.row(0).toVector3d().times(side))
                torsoPos = torsoPos.plus(moveOrientation.row(1).toVector3d().times(up))
                torsoPos = torsoPos.plus(moveOrientation.row(2).toVector3d().times(forward))
            }

            function rotateYaw(yaw, inp) {
                var yawMat = Qt.matrix4x4(
                    Math.cos(yaw),    0.0, -Math.sin(yaw),    0.0,
                    0.0,              1.0, 0.0,               0.0,
                    Math.sin(yaw),    0.0, Math.cos(yaw),     0.0,
                    0.0,              0.0, 0.0,               1.0)
                if(fixedUpVector)
                    return yawMat.times(inp);
                else
                    return headOrientation.times(yawMat.times(headOrientationInverse.times(inp)))
            }
            function rotatePitch(pitch, inp) {
                if(fixedUpVector) return inp;
                var pitchMat = Qt.matrix4x4(
                   1.0,              0.0, 0.0,               0.0,
                   0.0,  Math.cos(pitch),  -Math.sin(pitch), 0.0,
                   0.0,  Math.sin(pitch),   Math.cos(pitch), 0.0,
                   0.0,              0.0, 0.0              , 1.0)
                return headOrientation.times(pitchMat.times(headOrientationInverse.times(inp)))
            }
            function rotateBank(bank, inp) {
                if(fixedUpVector) return inp;
                var bankMat = Qt.matrix4x4(
                   Math.cos(bank), -Math.sin(bank),  0.0, 0.0,
                   Math.sin(bank),  Math.cos(bank),  0.0, 0.0,
                   0.0,             0.0,             1.0, 0.0,
                   0.0,             0.0,             0.0, 1.0)
                return headOrientation.times(bankMat.times(headOrientationInverse.times(inp)))
            }

            // prohibit banking
            function fixUpvector(inp) {
                // Not yet working
                var side // not needed for read
                var upvec = Qt.vector3d(0.0, 1.0, 0.0); // constrain upvector
                var forward = inp.row(2).toVector3d() // keep forward direction

                // orthonormalize
                side = upvec.crossProduct(forward).normalized()
                upvec = forward.crossProduct(side).normalized()
                forward = side.crossProduct(upvec).normalized()
                inp.m11 = side.x     ; inp.m12 = side.y     ; inp.m13 = side.z
                inp.m21 = upvec.x    ; inp.m22 = upvec.y    ; inp.m23 = upvec.z
                inp.m31 = forward.x  ; inp.m32 = forward.y  ; inp.m33 = forward.z
                return inp
            }
            MouseArea {
                id: screenMouse
                anchors.fill: parent
                hoverEnabled: true
                acceptedButtons: Qt.LeftButton | Qt.MiddleButton | Qt.RightButton
                property bool rotating: false
                property bool translating: false
                property real mx
                property real my
                onWheel: {
                    drawingArea.move(0.0, 0.0, wheel.angleDelta.y*0.01);
                }
                function pressedButtonsChanged() {
                    var leftButton = screenMouse.pressedButtons & Qt.LeftButton
                    var rightButton = screenMouse.pressedButtons & Qt.RightButton
                    var middleButton = screenMouse.pressedButtons & Qt.MiddleButton
                    if(leftButton && !rightButton) {
                        rotating = true
                        translating = false
                    } else if (middleButton || leftButton && rightButton) {
                        rotating = false
                        translating = true
                    } else {
                        rotating = false
                        translating = false
                    }
                }
                onPressed: {
                    mx = mouseX
                    my = mouseY
                    pressedButtonsChanged()
                }
                onReleased: {
                    pressedButtonsChanged()
                }
                onPositionChanged: {
                    var movementx = (mx - mouseX) * 0.05
                    var movementy = (my - mouseY) * 0.05
                    if(rotating) {
                        var tmp = drawingArea.torsoOrientation
                        tmp = drawingArea.rotateYaw(movementx*0.1, tmp)
                        tmp = drawingArea.rotatePitch(-movementy*0.1, tmp)
                        if(drawingArea.fixedUpVector)
                        {
                            tmp = drawingArea.fixUpvector(tmp)
                        }
                        drawingArea.torsoOrientation = tmp
                    } else if(translating) {
                        drawingArea.move(movementx, movementy, 0.0)
                    }
                    mx = mouseX
                    my = mouseY
                }
                focus: true
                Keys.onPressed: {
                    var speed = xboxController.speed;
                    if (event.key === Qt.Key_Shift) {
                        speed *= 2;
                    }
                    if (event.key === Qt.Key_Left || event.key === Qt.Key_A) {
                        drawingArea.move(speed, 0.0, 0.0);
                    }
                    if (event.key === Qt.Key_Right || event.key === Qt.Key_D) {
                        drawingArea.move(-speed, 0.0, 0.0);
                    }
                    if (event.key === Qt.Key_Up || event.key === Qt.Key_W) {
                        drawingArea.move(0.0, 0.0, speed);
                    }
                    if (event.key === Qt.Key_Down || event.key === Qt.Key_S) {
                        drawingArea.move(0.0, 0.0, -speed);
                    }
                    if (event.key === Qt.Key_Y) {
                        drawingArea.move(0.0, -speed, 0.0);
                    }
                    if (event.key === Qt.Key_X) {
                        drawingArea.move(0.0, speed, 0.0);
                    }
                    var bank = 0;
                    if (event.key === Qt.Key_Q) {
                        bank = -0.1;
                    }
                    if (event.key === Qt.Key_E) {
                        bank = 0.1;
                    }
                    if (bank !== 0) {
                        var tmp = drawingArea.torsoOrientation
                        tmp = drawingArea.rotateBank(bank, tmp)
                        drawingArea.torsoOrientation = tmp
                    }
                    if (event.key === Qt.Key_H) {
                        drawingArea.pointSize++
                    }
                    if (event.key === Qt.Key_J) {
                        drawingArea.pointSize = Math.max(1.0,drawingArea.pointSize-1);
                    }
                }
                XBoxController {
                    property int demoPoint: 0
                    property var demoPoints: [
                        Qt.vector3d(93.25, -44.73, 1.38),
                        Qt.vector3d(90.464, 6.01, 1.328),
                        Qt.vector3d(21.472, 4.797, 51.95),
                        Qt.vector3d(-29.691, 2.997, 32.143),
                        Qt.vector3d(-18.7, -4.452, 13.941),
                        Qt.vector3d(-67.83, -3.13, 54.82),
                        Qt.vector3d(9.6, -7.84, 39.78)
                    ]
                    property real speed: (buttonB?0.01:0.1) + buttonA * 1.0
                    property real movForward: (stickRY + triggerRight - triggerLeft) * speed
                    property real movRight: stickRX*speed
                    property real rotY: stickLY*-0.02 * (1.0 + invertY.checked * -2.0)
                    property real rotX: stickLX*-0.02
                    property real pointSizeGrowth: dpadRight - dpadLeft
                    property real distanceDetailChange: dpadUp - dpadDown
                    property real distanceDetailInv: 1.0
                    id: xboxController
                    onButtonStartChanged: {
                        if(buttonStart) {
                            drawingArea.vrmode = !drawingArea.vrmode
                        }
                    }
                    onLeftShoulderChanged: {
                        if(buttonBack) {
                            invertY.checked = !invertY.checked
                        }
                    }
                    onLeftThumbChanged: {
                        if(leftThumb) {
                            fixedUpvec.checked = !fixedUpvec.checked
                        }
                    }
                    onButtonBackChanged: {
                        if(buttonBack) {
                            demoPoint++
                            if(demoPoint >= demoPoints.length) {
                                demoPoint = 0
                            }
                            drawingArea.pointSize = 64.0
                            if(demoPoint == 0) {
                                drawingArea.distanceDetail = 10.0
                            } else {
                                drawingArea.distanceDetail = 1.0
                            }
                            drawingArea.torsoPos = demoPoints[demoPoint]
                        }
                    }
                    Component.onCompleted: {
                        drawingArea.torsoPos = demoPoints[demoPoint]
                    }
                    function onFrame() {
                        update()
                        drawingArea.pointSize += pointSizeGrowth
                        distanceDetailInv += distanceDetailChange*0.1
                        distanceDetailInv = Math.max(0.01, distanceDetailInv)
                        drawingArea.distanceDetail = 1.0/distanceDetailInv
                        drawingArea.move(-movRight, 0.0, movForward)

                        var tmp = drawingArea.torsoOrientation
                        tmp = drawingArea.rotateYaw(rotX, tmp)
                        tmp = drawingArea.rotatePitch(rotY, tmp)
                        if(drawingArea.fixedUpVector)
                        {
                            tmp = drawingArea.fixUpvector(tmp)
                        }
                        drawingArea.torsoOrientation = tmp
                    }
                }
            }
        }
    }

    SystemPalette {
        id: palette
    }
}
