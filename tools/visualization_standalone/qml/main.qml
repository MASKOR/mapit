import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1
import QtGraphicalEffects 1.0
import QtQuick.Scene3D 2.0
import Qt3D.Core 2.0 as Q3D
import Qt3D.Render 2.0
import Qt3D.Input 2.0
//import Qt3D.Extras 2.0

import fhac.upns 1.0 as UPNS
//import "components"
//import "operators"

import QtQuick 2.0 as QQ2

ApplicationWindow {
    title: qsTr("Map Visualization")
    width: 1200
    height: 800
    visible: true
    menuBar: MainMenubar {
        id: menubar
        //uiEnabled: drawingArea.renderdata.running
    }
    GridLayout {
        UPNS.Repository {
            id: repo
            conf: "./repo.yaml"
        }
        anchors.fill: parent
        Scene3D {
            id: scene3d
            Layout.minimumWidth: 50
            Layout.fillWidth: true
            Layout.fillHeight: true
            aspects: ["render", "logic", "input"]
            focus: true

            Q3D.Entity {
                id: sceneRoot
                Q3D.Camera {
                    id: camera
                    projectionType: Q3D.CameraLens.PerspectiveProjection
                    fieldOfView: 45
                    aspectRatio: scene3d.width/scene3d.height
                    nearPlane : 0.1
                    farPlane : 1000.0
                    position: Qt.vector3d( 0.0, 0.0, -40.0 )
                    upVector: Qt.vector3d( 0.0, 1.0, 0.0 )
                    viewCenter: Qt.vector3d( 0.0, 0.0, 0.0 )
                }

                Q3D.Configuration  {
                    controlledCamera: camera
                }

                components: [
                    FrameGraph {
                        activeFrameGraph: Viewport {
                            id: viewport
                            rect: Qt.rect(0.0, 0.0, 1.0, 1.0) // From Top Left
                            clearColor: "transparent"

                            CameraSelector {
                                id : cameraSelector
                                camera: camera
                                ClearBuffer {
                                    buffers : ClearBuffer.ColorDepthBuffer
                                    LayerFilter {
                                        layers: ["solid"]
                                    }
                                }
                                LayerFilter {
                                    layers: ["points"]
                                    StateSet {
                                        renderStates: [
                                            PointSize { specification: PointSize.StaticValue; value: 5 }
                                        ]
                                    }
                                }
                            }
                        }
                    }
                ]

                Q3D.Entity {
                    id: pointcloud
                    property Layer layerPoints: Layer {
                            names: "points"
                        }
                    property var meshTransform: Q3D.Transform {
                            property real userAngle: 0.0
                            scale: 10
                            rotation: fromAxisAndAngle(Qt.vector3d(0, 1, 0), userAngle)
                        }
                    property GeometryRenderer customMesh: UPNS.EntitydataRenderer {
                            entitydata: repo.getCheckout("testcheckout").getEntitydataReadOnly("testmap/testlayer/bunnyNormalEst")
                        }
                    property Material materialPoint: PerVertexColorMaterial {}
                    components: [ customMesh, materialPoint, meshTransform, layerPoints ]
                }

                Q3D.Entity {
                    id: customEntity
                    property Layer layerPoints: Layer {
                            names: "noneZ"
                        }
                    property var meshTransform: Q3D.Transform {
                            property real userAngle: 0.0
                            scale: 10
                            rotation: fromAxisAndAngle(Qt.vector3d(0, 1, 0), userAngle)
                        }
                    property GeometryRenderer customMesh: GeometryRenderer {
                            instanceCount: 1
                            baseVertex: 0
                            baseInstance: 0
                            primitiveType: GeometryRenderer.Points
                            Buffer {
                                id: vertexBuffer
                                type: Buffer.VertexBuffer
                                data: {
                                        // Vertices
                                        var v0 = Qt.vector3d(-1.0, 0.0, -1.0)
                                        var v1 = Qt.vector3d(1.0, 0.0, -1.0)
                                        var v2 = Qt.vector3d(0.0, 1.0, 0.0)
                                        var v3 = Qt.vector3d(0.0, 0.0, 1.0)

                                        // Face Normals
                                        function normal(v0, v1, v2) {
                                            return v1.minus(v0).crossProduct(v2.minus(v0)).normalized();
                                        }
                                        var n023 = normal(v0, v2, v3)
                                        var n012 = normal(v0, v1, v2)
                                        var n310 = normal(v3, v1, v0)
                                        var n132 = normal(v1, v3, v2)

                                        // Vector normals
                                        var n0 = n023.plus(n012).plus(n310).normalized()
                                        var n1 = n132.plus(n012).plus(n310).normalized()
                                        var n2 = n132.plus(n012).plus(n023).normalized()
                                        var n3 = n132.plus(n310).plus(n023).normalized()

                                        // Colors
                                        var red = Qt.vector3d(1.0, 0.0, 0.0)
                                        var green = Qt.vector3d(0.0, 1.0, 0.0)
                                        var blue = Qt.vector3d(0.0, 0.0, 1.0)
                                        var white = Qt.vector3d(1.0, 1.0, 1.0)

                                        var vertices = [
                                                    v0, n0, red,
                                                    v1, n1, blue,
                                                    v2, n2, green,
                                                    v3, n3, white
                                                ]

                                        var vertexArray = new Float32Array(4 * (3 + 3 + 3));
                                        var i = 0;

                                        vertices.forEach(function(vec3) {
                                            vertexArray[i++] = vec3.x;
                                            vertexArray[i++] = vec3.y;
                                            vertexArray[i++] = vec3.z;
                                        });

                                        return vertexArray;
                                    }
                            }

                            geometry:  Geometry {
                                Attribute {
                                    attributeType: Attribute.VertexAttribute
                                    dataType: Attribute.Float
                                    dataSize: 3
                                    byteOffset: 0
                                    byteStride: 9 * 4
                                    count: 4
                                    name: defaultPositionAttributeName()
                                    buffer: vertexBuffer
                                }

                                Attribute {
                                    attributeType: Attribute.VertexAttribute
                                    dataType: Attribute.Float
                                    dataSize: 3
                                    byteOffset: 3 * 4
                                    byteStride: 9 * 4
                                    count: 4
                                    name: defaultNormalAttributeName()
                                    buffer: vertexBuffer
                                }

                                Attribute {
                                    attributeType: Attribute.VertexAttribute
                                    dataType: Attribute.Float
                                    dataSize: 3
                                    byteOffset: 6 * 4
                                    byteStride: 9 * 4
                                    count: 4
                                    name: defaultColorAttributeName()
                                    buffer: vertexBuffer
                                }
                            }
                        }
                    property Material materialPoint: PerVertexColorMaterial {}
                    components: [ customMesh, materialPoint, meshTransform, layerPoints ]
                }


                Q3D.Entity {
                    id: sphereEntity
                    property Layer layerSolid: Layer {
                            names: "solid"
                        }
                    property var meshTransform: Q3D.Transform {
                        id: sphereTransform
                        property real userAngle: 0.0
                        translation: Qt.vector3d(20, 0, 0)
                        rotation: fromAxisAndAngle(Qt.vector3d(0, 1, 0), userAngle)
                    }
                    property var sphereMesh: SphereMesh {
                        radius: 3
                    }
                    property Material materialPhong: PhongMaterial {
                    }
                    components: [ sphereMesh, materialPhong, meshTransform, layerSolid ]
                }
                NumberAnimation {
                    target: sphereTransform
                    property: "userAngle"
                    duration: 10000
                    from: 0
                    to: 360

                    loops: Animation.Infinite
                    running: true
                }
            }
        }
/*        MapsRenderViewport {
            id: drawingArea
            Layout.minimumWidth: 50
            Layout.fillWidth: true
            Layout.fillHeight: true
//            entitydata: EntityDataPointcloud2 {
//                filename: "data/fh/all_pointclouds20_norm_flipped.pcd"
//            }
            renderdata.entitydata: repo.getCheckout("testcheckout").getEntitydataReadOnly("testmap/testlayer/bunnyNormalEst")
//            renderdata.filename: "data/fh/all_pointclouds20_norm_flipped.pcd"
            //renderdata.filename: "data/Rover.pcd"
            renderdata.vrmode: menubar.enableVr
            Component.onCompleted: {
                if(renderdata.vrmode === false) {
                    menubar.vrAvailable = false
                    menubar.enableVr = false
                    menubar.fixUpvector = false
                }
            }
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
            AxisGizmo {
                id: gizmo
                anchors.right: parent.right
                anchors.bottom: parent.bottom
                anchors.margins: 10
                width: 100
                height: 100
                finalTransform: drawingArea.finalTransform
                property bool showing: true
                opacity: showing
                Behavior on opacity {
                    NumberAnimation {
                        duration: 200
                    }
                }
            }
            Text {
                id: f1info
                text: "F1: " + (hud.showing?"Hide":"Show") + " Help"
                state: hud.showing?"Show":"Hide"
                states: [
                        State {name: "Show"},
                        State {name: "Hide"}
                    ]
                transitions: [
                        Transition {
                            id: hideHudTransition
                            to: "Hide"
                            ParallelAnimation {
                                ColorAnimation {
                                    target:f1info;
                                    property: "color"
                                    to: "white"
                                    duration: 200
                                }
                                SequentialAnimation {
                                    NumberAnimation {
                                        target:f1info;
                                        property:"opacity"
                                        to:1.0
                                        duration: 5000
                                    }
                                    NumberAnimation {
                                        target:f1info;
                                        property:"opacity"
                                        to:0.0
                                        duration: 1000
                                    }
                                }
                            }
                        },
                        Transition {
                            to: "Show"
                            ParallelAnimation {
                                ColorAnimation {
                                    target:f1info;
                                    property: "color"
                                    to: "grey"
                                    duration: 200
                                }
                                NumberAnimation {
                                    target:f1info;
                                    property:"opacity"
                                    to: 1.0
                                    duration: 200
                                }
                            }
                        }
                    ]
            }
            Item {
                id: hud
                anchors.top: f1info.bottom
                anchors.left: parent.left
                anchors.right: parent.right
                property bool showing: true
                opacity: showing
                Component.onCompleted: {
                    showing = false
                }

                Behavior on opacity {
                    NumberAnimation { duration: 200 }
                }
                Text {
                    anchors.top: hud.top
                    id: controllerText
                    text: "Controller " + (xboxController.controllerId==-1?"not ":"") + "found"
                    color: xboxController.controllerId==-1?"red":"green"
                }
                Text {
                    id: posText
                    anchors.top: controllerText.bottom
                    color: "white"
                    text: "Pos: (" + camera.torsoPos.x + ", " + camera.torsoPos.y + ", " + camera.torsoPos.z + ")"
                }
                Text {
                    id: help
                    anchors.top: posText.bottom
                    //font.bold: true
                    style: Text.Outline
                    styleColor: "black"
                    textFormat: Text.RichText
                    text: "\n<br/><b>Controller:</b>\n<br/>" +
                          "L-Stick: Rotate\n<br/>" +
                          "R-Stick: Move\n<br/>" +
                          "L-Trigger: Forward\n<br/>" +
                          "R-Trigger: Backward\n<br/>" +
                          "Select: Go to Demopoint and Reset (Demopoint: <font color=\"#00FF00\">" + (xboxController.demoPoint + 1) + "</font>)\n<br/>" +
                          "DPad Right/Left: Change Pointsize\n<br/>" +
                          "DPad Up/Down: Change LOD (<font color=\"#00FF00\">" + drawingArea.renderdata.distanceDetail.toFixed(1) + "</font>)\n<br/>" +
                          "A-Button: Move Faster\n" +
                          "LeftThumb: Fix Upvector (<font color=\"#00FF00\">" + (menubar.fixUpvector?"":"not ") + "fixed</font>)\n<br/>" +
                          "Start: <font color=\"#00FF00\">" + (drawingArea.renderdata.vrmode?"Disable":"Enable") + "</font> VR\n<br/>" +
                          "Left Shoulder Button: Invert Y Axis for Stick (<font color=\"#00FF00\">" + (menubar.fixUpvector?"fixed":((menubar.invertYAxis?"":"not ") + "inverted")) + "</font>)\n\n<br/><br/>" +
                          "<b>Keyboard:</b>\n<br/>" +
                          "F2: <font color=\"#00FF00\">"+(gizmo.showing?"Hide":"Show")+"</font> Gizmo\n<br/>" +
                          "F3: <font color=\"#00FF00\">"+ (menubar.showCenterCross?"Hide":"Show") +"</font> Center Cross\n<br/>" +
                          "Shift: Move Faster\n<br/>" +
                          "Left/Right/UpDown/W/A/S/D: Move\n<br/>" +
                          "Y/X: Move Up/Down\n<br/>" +
                          "Q/E: Rotate (Banking)\n<br/>" +
                          "H/J: Change Point Size (<font color=\"#00FF00\">" + drawingArea.renderdata.pointSize.toFixed(2) + "</font>)\n<br/>" +
                          "K/L: Lod rate (<font color=\"#00FF00\">" + screenMouse.distanceDetailKeyboardTemp + "</font>)\n<br/>" +
                          "M: Change Point Render Mode (<font color=\"#00FF00\">" + screenMouse.renderModeName + "</font>)\n<br/>" +
                          "B/N: Change Fov (<font color=\"#00FF00\">" + drawingArea.renderdata.fov.toFixed(1) + "°</font>)\n\n<br/><br/>" +
                          "<b>Mouse:</b>\n<br/>" +
                          "Middle: Move perpendicular to view direction\n<br/>" +
                          "Wheel: Zoom"
                    color: "white"
                }
                Rectangle {
                    anchors.fill: help
                    z: help.z - 1
                    anchors.margins: -5
                    color: Qt.rgba(0, 0, 0, 0.5)
                }
//                FastBlur {
//                    anchors.fill: help
//                    source: drawingArea
//                    radius: 64
//                    cached: false
//                }
//                Text {
//                    x: help.x+2
//                    y: help.y+2
//                    z: help.z-1
//                    font.bold: true
//                    text: help.text
//                    color: "black"
//                }
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
                property var distanceDetailKeyboardTemp: 0.0
                property var renderModeName: "disc: ray-cast (FH Aachen)"
                Keys.onPressed: {
                    var speed = xboxController.speed;
                    if (event.key === Qt.Key_F1) {
                        hud.showing = !hud.showing
                    }
                    if (event.key === Qt.Key_F2) {
                        gizmo.showing = !gizmo.showing
                    }
                    if (event.key === Qt.Key_F3) {
                        menubar.showCenterCross = !menubar.showCenterCross
                    }
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
                        drawingArea.renderdata.pointSize += 0.01
                    }
                    if (event.key === Qt.Key_J) {
                        drawingArea.renderdata.pointSize = Math.max(0.005,drawingArea.renderdata.pointSize-0.005);
                    }
                    if (event.key === Qt.Key_K) {
                        distanceDetailKeyboardTemp--;
                    }
                    if (event.key === Qt.Key_L) {
                        distanceDetailKeyboardTemp++;
                    }
                    if (event.key === Qt.Key_M) {
                        drawingArea.renderdata.disc = (drawingArea.renderdata.disc+1)%8;
                        var cons = drawingArea.renderdata.disc > 3
                        if(cons) {
                            renderModeName = "constant ";
                        } else {
                            renderModeName = "perspective scaled ";
                        }

                        if(drawingArea.renderdata.disc%4 == 0) {
                            //if(cons) drawingArea.renderdata.disc++;
                            renderModeName += "disc: ray-cast (FH Aachen)";
                        }
                        if(drawingArea.renderdata.disc%4 == 1) {
                            renderModeName += "disc: circle";
                        }
                        if(drawingArea.renderdata.disc%4 == 2) {
                            renderModeName += "disc: ellipse";
                        }
                        if(drawingArea.renderdata.disc%4 == 3) {
                            renderModeName += "disc: square";
                        }
                        console.log(renderModeName);
                    }
                    if (event.key === Qt.Key_B) {
                        console.log("fov: " + drawingArea.renderdata.fov);
                        drawingArea.renderdata.fov++;
                    }
                    if (event.key === Qt.Key_N) {
                        console.log("fov: " + drawingArea.renderdata.fov);
                        drawingArea.renderdata.fov--;
                    }
                }

                XBoxController {
                    id: xboxController
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
                    property real pointSizeGrowth: (dpadRight - dpadLeft) * 0.05
                    property real distanceDetailChange: dpadUp - dpadDown + screenMouse.distanceDetailKeyboardTemp
                    property real distanceDetailInv: 1.0
                    onButtonStartChanged: {
                        if(buttonStart) {
                            drawingArea.renderdata.vrmode = !drawingArea.renderdata.vrmode
                        }
                    }
                    onLeftShoulderChanged: {
                        if(leftShoulder) {
                            menubar.invertYAxis = !menubar.invertYAxis
                        }
                    }
                    onLeftThumbChanged: {
                        if(leftThumb) {
                            menubar.fixUpvector = !menubar.fixUpvector
                        }
                    }
                    onButtonBackChanged: {
                        if(buttonBack) {
                            demoPoint++
                            if(demoPoint >= demoPoints.length) {
                                demoPoint = 0
                            }
                            drawingArea.renderdata.pointSize = 0.064
                            if(demoPoint == 0) {
                                drawingArea.renderdata.distanceDetail = 10.0
                            } else {
                                drawingArea.renderdata.distanceDetail = 1.0
                            }
                            screenMouse.distanceDetailKeyboardTemp = 0.0
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
*/
    }

//    Timer {
//        id: tmr
//        repeat: true
//        interval: 2000
//        running: false
//        onTriggered: {
//            if( drawingArea.renderdata.disc === 0 ){
//                drawingArea.renderdata.disc = 1
//            } else if( drawingArea.renderdata.disc === 1 ){
//                drawingArea.renderdata.disc = 2
//            } else if( drawingArea.renderdata.disc === 2 ) {
//                drawingArea.renderdata.disc = 0
//            }
//        }

//    }

    SystemPalette {
        id: palette
    }
}
