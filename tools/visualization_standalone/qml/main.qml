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
            MenuItem {
                id: detailHiItem
                text: qsTr("&Hi Detail")
                checkable: true
                checked: true
                onCheckedChanged: {
                }
            }
            MenuItem {
                id: detailMidtem
                text: qsTr("&Mid Detail")
                checkable: true
                checked: false
                onCheckedChanged: {
                }
            }
            MenuItem {
                id: detailLoItem
                text: qsTr("&Low Detail")
                checkable: true
                checked: false
                onCheckedChanged: {
                }
            }
            MenuSeparator { }
            MenuItem {
                id: executeItem
                text: qsTr("Lower Detail when &moving")
                enabled: false
                onTriggered: {
                }
            }
            MenuSeparator { }
            MenuItem {
                id: showCenter
                text: qsTr("Show Center &Cross")
                checkable: true
                checked: true
            }
            MenuSeparator { }
            MenuItem {
                id: centerFixed
                text: qsTr("Center fixed to view")
                checkable: true
                checked: true
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
            //mapManager: Globals._mapManager
            //mapId: mapsList.currentMapId
            //layerId: mapLayers.currentLayerId
            entitydata: EntityDataPointcloud2 {
                //filename: "data/bunny.pcd"
                //filename: "data/alignedRGB_FH/Aligned_FARO_Scan_072.ply"
                filename: "data/fh/all.pcd"
            }
            vrmode: true
            property var pos: Qt.vector3d(0.0,0.0,0.0)
            property var offsX: 0.0
            property var offsY: 0.0
            property real angleX: 0
            property real angleY: 0
            property real zoom: 1.0
            Behavior on angleX {
                SmoothedAnimation { velocity: 10.0; reversingMode: SmoothedAnimation.Immediate }
            }
            Behavior on angleY {
                SmoothedAnimation { velocity: 10.0; reversingMode: SmoothedAnimation.Immediate }
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
                text: "Pos: " + drawingArea.torsoPos.x + ", " + drawingArea.torsoPos.y + ", " + drawingArea.torsoPos.z + ")"
            }

//            property var rotX: Qt.matrix4x4(
//                                   Math.cos(angleX), 0.0, -Math.sin(angleX), 0.0,
//                                   0.0,              1.0, 0.0,               0.0,
//                                   Math.sin(angleX), 0.0, Math.cos(angleX),  0.0,
//                                   0.0,              0.0, 0.0,               1.0)
//            property var rotY: Qt.matrix4x4(
//                                   1.0,              0.0, 0.0,               0.0,
//                                   0.0, Math.cos(angleY), -Math.sin(angleY), 0.0,
//                                   0.0, Math.sin(angleY),  Math.cos(angleY), 0.0,
//                                   0.0,              0.0, 0.0              , 1.0)
//            // center of rotation
//            property var posMat: Qt.matrix4x4(
//                                     1.0, 0.0, 0.0, pos.x,
//                                     0.0, 1.0, 0.0, pos.y,
//                                     0.0, 0.0, 1.0, pos.z,
//                                     0.0, 0.0, 0.0, 1.0)
//            // offset while translating
//            property var offsMat: Qt.matrix4x4(
//                                     1.0, 0.0, 0.0, offsX,
//                                     0.0, 1.0, 0.0, offsY,
//                                     0.0, 0.0, 1.0, 0.0,
//                                     0.0, 0.0, 0.0, 1.0)
//            property var zoomMat: Qt.matrix4x4(
//                                   zoom, 0.0,  0.0, 0.0,
//                                   0.0,  zoom, 0.0, 0.0,
//                                   0.0,  0.0, zoom, 0.0,
//                                   0.0,  0.0,  0.0, 1.0)
//            matrix: zoomMat.times(offsMat).times(rotY).times(rotX).times(posMat)

            property bool moveAlongHeadDirection: true // move where head shows or where torso shows?
            property bool fixedUpVector: true
            property var upVector: Qt.vector3d(0.0,1.0,0.0)
            property var forwardVector: Qt.vector3d(0.0,0.0,-1.0)
            property var rightVector: upVector.crossProduct(forwardVector)
//            property var rightVector: Qt.vector3d(upVector.y*forwardVector.z - upVector.z*forwardVector.y,
//                                                  upVector.z*forwardVector.x - upVector.x*forwardVector.z,
//                                                  upVector.x*forwardVector.y - upVector.y*forwardVector.x)
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
//                                     rightVector.x,     rightVector.y,      rightVector.z,   0.0,
//                                     upVector.x,        upVector.y,         upVector.z,      0.0,
//                                     forwardVector.x,   forwardVector.y,    forwardVector.z, 0.0,
//                                     0.0,               0.0,                0.0,             1.0)

            // inverse head matrix
            property var headOrientationInverse: headOrientation.transposed()

            // orientation to yaw and pitch around (always view)
            property var rotationOrientation: headOrientationInverse.times(torsoOrientation)

            // orientation to move along (may be torso or head)
            property var moveOrientation: moveAlongHeadDirection?rotationOrientation:torsoOrientation

            // output
            matrix: torsoOrientation.times(torsoPositionMatrix);

            function move(side, up, forward) {
                torsoPos = torsoPos.plus(moveOrientation.row(0).toVector3d().times(-side))
                torsoPos = torsoPos.plus(moveOrientation.row(1).toVector3d().times(up))
                torsoPos = torsoPos.plus(moveOrientation.row(2).toVector3d().times(forward))
            }

            function rotateYaw(yaw) {
                var yawMat = Qt.matrix4x4(
                    Math.cos(yaw),    0.0, -Math.sin(yaw),    0.0,
                    0.0,              1.0, 0.0,               0.0,
                    Math.sin(yaw),    0.0, Math.cos(yaw),     0.0,
                    0.0,              0.0, 0.0,               1.0)
                torsoOrientation = headOrientation.times(yawMat.times(headOrientationInverse.times(torsoOrientation)))

                //forwardVector = yawMat.times(forwardVector.toVector4d()).toVector3d()
                // fore_(i+1) = inv(head) * rot * head * fore_i
//                forwardVector = headOrientation.times(yawMat.times(headOrientationInverse.times(forwardVector.toVector4d()))).toVector3d()
//                forwardVector = forwardVector.normalized()
            }
            function rotatePitch(pitch) {
                var pitchMat = Qt.matrix4x4(
                   1.0,              0.0, 0.0,               0.0,
                   0.0,  Math.cos(pitch),  -Math.sin(pitch), 0.0,
                   0.0,  Math.sin(pitch),   Math.cos(pitch), 0.0,
                   0.0,              0.0, 0.0              , 1.0)
                torsoOrientation = headOrientation.times(pitchMat.times(headOrientationInverse.times(torsoOrientation)))
//                //forwardVector = pitchMat.times(forwardVector.toVector4d()).toVector3d()
//                forwardVector = headOrientation.times(pitchMat.times(headOrientationInverse.times(forwardVector.toVector4d()))).toVector3d()
//                //forwardVector = headOrientationInverse.times(pitchMat).times(rotationOrientation.row(2)).toVector3d()
//                upVector = forwardVector.crossProduct(rightVector)
//                forwardVector = forwardVector.normalized()
//                upVector = upVector.normalized()
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
                property real ax
                property real ay
                onWheel: {
                    drawingArea.move(0.0, 0.0, wheel.angleDelta.y*0.001);
                    //drawingArea.zoom *= Math.pow(2.0, wheel.angleDelta.y*0.001)
                }
                function pressedButtonsChanged() {
                    var leftButton = screenMouse.pressedButtons & Qt.LeftButton
                    var rightButton = screenMouse.pressedButtons & Qt.RightButton
                    var middleButton = screenMouse.pressedButtons & Qt.MiddleButton
                    if(leftButton && !rightButton) {
                        rotating = true
                        translating = false
                    } else if(middleButton || leftButton && rightButton) {
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
                    ax = drawingArea.angleX
                    ay = drawingArea.angleY
                    pressedButtonsChanged()
                }
                onReleased: {
                    pressedButtonsChanged()
                }
                onTranslatingChanged: {
                    if(!translating) {
                        if(drawingArea.offsX !== 0.0 || drawingArea.offsY !== 0.0) {
//                            console.log("moving: " + drawingArea.offsX + ", " + drawingArea.offsY);
//                            drawingArea.move(drawingArea.offsX, drawingArea.offsY, 0.0);
//                            var finalRotationMatrix = drawingArea.rotY.times(drawingArea.rotX)
//                            var posV4 = finalRotationMatrix.inverted().times(Qt.vector4d(drawingArea.offsX,
//                                                                              drawingArea.offsY,
//                                                                                            0.0,
//                                                                                            1.0))
//                            drawingArea.pos = drawingArea.pos.plus(Qt.vector3d(posV4.x, posV4.y, posV4.z))

                            drawingArea.offsX = 0.0
                            drawingArea.offsY = 0.0
                        }
                    }
                }
                onPositionChanged: {
                    var movementx = (mx - mouseX) * 0.05
                    var movementy = (my - mouseY) * 0.05
                    if(rotating) {
                        drawingArea.rotateYaw(-movementx*0.1, 0.0, 0.0);
                        drawingArea.rotatePitch(movementy*0.1, 0.0, 0.0);
//                        drawingArea.angleX = ax + movementx
//                        drawingArea.angleY = ay + movementy
                    } else if(translating) {
                        drawingArea.move(movementx, movementy, 0.0);
//                        drawingArea.offsX = movementx*-0.1
//                        drawingArea.offsY = movementy*-0.1
                    }
                    mx = mouseX
                    my = mouseY
                }
            }
        }
    }
    SystemPalette {
        id: palette
    }
}
