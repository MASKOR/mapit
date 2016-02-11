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
    width: 1200
    height: 800
    visible: true
    menuBar: MainMenubar {
        id: menubar
        uiEnabled: drawingArea.renderdata.running
    }
    GridLayout {
        anchors.fill: parent
        MapsRenderViewport {
            id: drawingArea
            Layout.minimumWidth: 50
            Layout.fillWidth: true
            Layout.fillHeight: true
//            entitydata: EntityDataPointcloud2 {
//                filename: "data/fh/all_pointclouds20_norm_flipped.pcd"
//            }
            renderdata.filename: "data/fh/all_pointclouds20_norm_flipped.pcd"
            //renderdata.filename: "data/Rover.pcd"
            renderdata.vrmode: menubar.enableVr
            renderdata.mirrorEnabled: menubar.mirrorEnabled
            renderdata.mirrorDistorsion: menubar.mirrorDistorsion
            renderdata.mirrorRightEye: menubar.mirrorRightEye

            onFrame: {
                xboxController.onFrame()
            }
            Rectangle {
                x: parent.width/2
                y: parent.height/2-5
                width: 1
                height: 11
                color: "white"
                visible: menubar.showCenterCross
            }
            Rectangle {
                x: parent.width/2-5
                y: parent.height/2
                width: 11
                height: 1
                color: "white"
                visible: menubar.showCenterCross
            }
            Text {
                color: "white"
                id: name
                text: "Pos: (" + camera.torsoPos.x + ", " + camera.torsoPos.y + ", " + camera.torsoPos.z + ")"
            }

            renderdata.matrix: camera.matrix
            Camera {
                id: camera
                fixedUpVector: menubar.fixUpvector
                headOrientation: drawingArea.renderdata.headOrientation
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
                    camera.move(0.0, 0.0, wheel.angleDelta.y*0.01);
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
                        camera.rotateYaw(movementx*0.1)
                        camera.rotatePitch(-movementy*0.1)
                        camera.finishMovement();
                    } else if(translating) {
                        camera.move(movementx, movementy, 0.0)
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
                        camera.move(speed, 0.0, 0.0);
                    }
                    if (event.key === Qt.Key_Right || event.key === Qt.Key_D) {
                        camera.move(-speed, 0.0, 0.0);
                    }
                    if (event.key === Qt.Key_Up || event.key === Qt.Key_W) {
                        camera.move(0.0, 0.0, speed);
                    }
                    if (event.key === Qt.Key_Down || event.key === Qt.Key_S) {
                        camera.move(0.0, 0.0, -speed);
                    }
                    if (event.key === Qt.Key_Y) {
                        camera.move(0.0, -speed, 0.0);
                    }
                    if (event.key === Qt.Key_X) {
                        camera.move(0.0, speed, 0.0);
                    }
                    var bank = 0;
                    if (event.key === Qt.Key_Q) {
                        bank = -0.1;
                    }
                    if (event.key === Qt.Key_E) {
                        bank = 0.1;
                    }
                    if (bank !== 0) {
                        tmp = camera.rotateBank(bank)
                        camera.finishMovement();
                    }
                    if (event.key === Qt.Key_H) {
                        drawingArea.renderdata.pointSize++
                    }
                    if (event.key === Qt.Key_J) {
                        drawingArea.renderdata.pointSize = Math.max(1.0,drawingArea.renderdata.pointSize-1);
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
                    property real rotY: stickLY*-0.02 * (1.0 + menubar.invertYAxis * -2.0)
                    property real rotX: stickLX*-0.02
                    property real pointSizeGrowth: dpadRight - dpadLeft
                    property real distanceDetailChange: dpadUp - dpadDown
                    property real distanceDetailInv: 1.0
                    id: xboxController
                    onButtonStartChanged: {
                        if(buttonStart) {
                            drawingArea.renderdata.vrmode = !drawingArea.renderdata.vrmode
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
                            drawingArea.renderdata.pointSize = 64.0
                            if(demoPoint == 0) {
                                drawingArea.renderdata.distanceDetail = 10.0
                            } else {
                                drawingArea.renderdata.distanceDetail = 1.0
                            }
                            camera.torsoPos = demoPoints[demoPoint]
                        }
                    }
                    Component.onCompleted: {
                        camera.torsoPos = demoPoints[demoPoint]
                    }
                    function onFrame() {
                        update()
                        drawingArea.renderdata.pointSize += pointSizeGrowth
                        distanceDetailInv += distanceDetailChange*0.1
                        distanceDetailInv = Math.max(0.01, distanceDetailInv)
                        drawingArea.renderdata.distanceDetail = 1.0/distanceDetailInv
                        camera.move(-movRight, 0.0, movForward)
                        camera.rotateYaw(rotX)
                        camera.rotatePitch(rotY)
                        camera.finishMovement()
                    }
                }
            }
        }
    }

    SystemPalette {
        id: palette
    }
}
