import QtQuick 2.4
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtQuick.Window 2.2 as Wnd
import QtGraphicalEffects 1.0
import QtQuick.Scene3D 2.0
import Qt3D.Core 2.0 as Q3D
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0

import pcl 1.0

import fhac.upns 1.0 as UPNS

import QtQuick 2.0 as QQ2

QCtl.ApplicationWindow {
    id: window
    title: qsTr("Map Visualization")
    width: 1200
    height: 800
    visible: true
    menuBar: MainMenubar {
        id: menubar
        //uiEnabled: drawingArea.renderdata.running
    }
//    UPNS.Repository {
//        id: repo
//        conf: "./repo.yaml"
//    }
    UPNS.Checkout {
        id: checkout
        repository: globalRepository
        name: "testcheckout"
        Component.onCompleted: {
            var ops = globalRepository.listOperators()
            globalRepository.listOperators().forEach(function(n){console.log(n)})
        }
    }

    RowLayout {
        anchors.fill: parent
        ColumnLayout {
            id: controlColumn
            Text { text: "PointSize: " + pointSizeSlider.value.toFixed(2) }
            QCtl.Slider {
                id: pointSizeSlider
                width: 100
                value: 0.5
                minimumValue: 0.01
                maximumValue:  2.0
            }
            AxisGizmo {
                height: controlColumn.width
                width: controlColumn.width
                finalTransform: mainCamera.viewMatrix
            }
            QCtl.Button {
                text: "Checkout"
                onClicked: chooseCheckoutDialog.visible = !chooseCheckoutDialog.visible
                Wnd.Window {
                    id: chooseCheckoutDialog
                    width: 420
                    height: 260
                    minimumHeight: height
                    maximumHeight: height
                    minimumWidth: width
                    maximumWidth: width
                    flags: Qt.Dialog
                    title: "Choose Checkout"
                    color: palette.window
                    ColumnLayout {
                        anchors.fill: parent

                        ListView {
                            id: checkoutList
                            delegate: Text {
                                    text: globalRepository.checkoutNames[index]
                                    color: palette.text
                                    MouseArea {
                                        anchors.fill: parent
                                        onClicked: checkoutList.currentIndex = index
                                    }
                                }

                            model: globalRepository.checkoutNames
                            highlight: Rectangle { color: palette.highlight }

                            Layout.fillWidth: true
                            Layout.fillHeight: true
                        }
                        QCtl.Button {
                            text: "+"
                            onClicked: {
                                newCheckoutDialog.visible = !newCheckoutDialog.visible
                            }
                            Wnd.Window {
                                id: newCheckoutDialog
                                width: 420
                                height: 260
                                minimumHeight: height
                                maximumHeight: height
                                minimumWidth: width
                                maximumWidth: width
                                flags: Qt.Dialog
                                title: "Choose Checkout"
                                color: palette.window
                                GridLayout {
                                    QCtl.Label {
                                        text: "Branchname"
                                        Layout.column: 0
                                        Layout.row: 0
                                    }
                                    QCtl.TextField {
                                        id: branchnameTextedit
                                        text: "master"
                                        Layout.column: 1
                                        Layout.row: 0
                                    }
                                    QCtl.Label {
                                        text: "Checkoutname"
                                        Layout.column: 0
                                        Layout.row: 1
                                    }
                                   QCtl. TextField {
                                        id: checkoutnameTextedit
                                        Layout.column: 1
                                        Layout.row: 1
                                    }
                                    QCtl.Button {
                                        text: "Cancel"
                                        onClicked: newCheckoutDialog.visible = false
                                        Layout.column: 0
                                        Layout.row: 2
                                    }
                                    QCtl.Button {
                                        text: "Ok"
                                        enabled: branchnameTextedit.text.trim().length !== 0
                                                 && checkoutnameTextedit.text.trim().length !== 0
                                        onClicked: {
                                            globalRepository.createCheckout(branchnameTextedit.text, checkoutnameTextedit.text)
                                            newCheckoutDialog.visible = false
                                        }
                                        Layout.column: 1
                                        Layout.row: 2
                                    }
                                }
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            QCtl.Button {
                                text: "Cancel"
                                onClicked: chooseCheckoutDialog.visible = false
                            }
                            QCtl.Button {
                                text: "Ok"
                                onClicked: {
                                    checkout.name = globalRepository.checkoutNames[checkoutList.currentIndex];
                                    chooseCheckoutDialog.visible = false;
                                }
                            }
                        }
                    }
                }
            }
            QCtl.Action {
                id: transformAction
                text: "&Transform"
                shortcut: "T"
                //iconName: "edit-copy"
                enabled: currentEntitydataTransform.path.length > 0
                //    target: "path/to/target.tf"
                //    mode: "relative"|"absolute",
                //    tf: [
                //      0: {mat: [0: m00, 1: m01, m02...], parent: "/path/to/parent", timestamp: unixts},
                //      1: {mat: [0: m00_2, ...]},
                //      2: ...
                //    ]
                // }
                onTriggered: {
                    console.log("executing");
                    checkout.doOperation("transform", {
                        target: currentEntitydataTransform.path,
                        mode: "absolute",
                        tf: {
                            mat:[100,   0,   0,   0,
                                   0, 100,   0,   0,
                                   0,   0, 100,   0,
                                   0,   0,   0,   1]
                        }
                    });
                }
            }
            QCtl.Menu {
                id: contextMenu
                QCtl.MenuItem {
                    action: transformAction
                }
            }

            QCtl.TreeView {
                id: treeViewCheckout
                model: UPNS.RootTreeModel {
                    root: checkout
                }
                QCtl.TableViewColumn {
                    role: "displayRole"
                    title: "Name"
                }
                QCtl.TableViewColumn {
                    id: pathColumn
                    role: "path"
                    title: "Path"
                }
                onCurrentIndexChanged: console.log(treeViewCheckout.model.data(treeViewCheckout.currentIndex, UPNS.RootTreeModel.NodeTypeRole));//treeViewCheckout.currentIndex.data(Qt.ToolTipRole));//treeViewCheckout.model.data(treeViewCheckout.currentIndex, Qt.ToolTipRole))
                rowDelegate: Item {
                    Rectangle {
                        anchors {
                            left: parent.left
                            right: parent.right
                            verticalCenter: parent.verticalCenter
                        }
                        height: parent.height
                        color: styleData.selected ? palette.highlight : palette.base
                        MouseArea {
                            anchors.fill: parent
                            acceptedButtons: Qt.RightButton
                            onClicked: {
                                if (mouse.button === Qt.RightButton)
                                {
                                    contextMenu.popup()
                                }
                            }
                        }
                    }
                }
            }
        }
        Layout.fillWidth: true

        MouseArea {
            id: sceneMouseArea
            Layout.minimumWidth: 50
            Layout.fillWidth: true
            Layout.fillHeight: true
            hoverEnabled: true
            property bool mouseOver: true
            onEntered: mouseOver = true
            onExited: {
                mouseOver = false
            }

            Scene3D {
                anchors.fill: parent
                id: scene3d
                aspects: ["render", "logic", "input"]
                focus: parent.mouseOver
                Q3D.Entity {
                    id: sceneRoot
                    Camera {
                        id: mainCamera
                        projectionType: CameraLens.PerspectiveProjection
                        fieldOfView: 45
                        aspectRatio: scene3d.width/scene3d.height
                        nearPlane : 0.1
                        farPlane : 1000.0
                        position: Qt.vector3d( 0.0, 0.0, -40.0 )
                        upVector: Qt.vector3d( 0.0, 1.0, 0.0 )
                        viewCenter: Qt.vector3d( 0.0, 0.0, 0.0 )
                    }

                    FirstPersonCameraController {
                    //OrbitCameraController {
                        id: cameraController
                        camera: mainCamera
                        linearSpeed: sceneMouseArea.mouseOver*12.0
                    }

                    components: [
                        RenderSettings {
                            activeFrameGraph: Viewport {
                                id: viewport
                                normalizedRect: Qt.rect(0.0, 0.0, 1.0, 1.0) // From Top Left
                                RenderSurfaceSelector {
                                    CameraSelector {
                                        id : cameraSelector
                                        camera: mainCamera
                                        FrustumCulling {
                                            ClearBuffers {
                                                buffers : ClearBuffers.ColorDepthBuffer
                                                clearColor: "white"
                                                NoDraw {}
                                            }
                                            LayerFilter {
                                                layers: solidLayer
                                            }
                                            LayerFilter {
                                                layers: pointLayer
                                                RenderStateSet {
                                                    renderStates: [
                                                        //PointSize { sizeMode: PointSize.Fixed; value: 5.0 }, // exception when closing application in qt 5.7
                                                        PointSize { sizeMode: PointSize.Programmable }, //supported since OpenGL 3.2
                                                        DepthTest { depthFunction: DepthTest.Less }
                                                        //DepthMask { mask: true }
                                                    ]
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        },
                        // Event Source will be set by the Qt3DQuickWindow
                        InputSettings {
                            eventSource: scene3d
                            enabled: true
                        }
                    ]

                    Q3D.Entity {
                        id: pointcloud
                        property Layer layerPoints: Layer {
                                id: pointLayer
                            }
                        property ObjectPicker picker: ObjectPicker {
                            onClicked: console.log("Clicked pcd", pick.distance, pick.triangleIndex)
                        }
                        property var meshTransform: Q3D.Transform {
                                id: theMeshTransform
                                //property real userAngle: -90.0
                                //scale: 10
                                //rotation: fromAxisAndAngle(Qt.vector3d(1, 0, 0), userAngle)
                                matrix: currentEntitydataTransform.matrix
                            }
                        UPNS.EntitydataTransform {
                            id: currentEntitydataTransform
                            checkout: checkout
                            property bool addTfToPath: currentEntitydata.path.length > 3 && currentEntitydata.path.lastIndexOf(".tf") !== currentEntitydata.path.length-3
                            path: currentEntitydata.path + (addTfToPath ? ".tf" : "")
                            onPathChanged: console.log("New Path of Tf is: " + path)
                        }
                        property GeometryRenderer customMesh: UPNS.EntitydataRenderer {
                                entitydata: UPNS.Entitydata {
                                    id: currentEntitydata
                                    checkout: checkout
                                    path: treeViewCheckout.currentIndex && treeViewCheckout.model.data(treeViewCheckout.currentIndex, UPNS.RootTreeModel.NodeTypeRole) === UPNS.RootTreeModel.EntityNode ? treeViewCheckout.model.data(treeViewCheckout.currentIndex, Qt.ToolTipRole) : ""// "corridor/lidar/pc1"
                                    onPathChanged: {
                                        console.log("Path of current entity: " + path)
                                    }
                                }
                            }
                        property Material materialPoint: Material {
                            effect: Effect {
                                techniques: Technique {
                                    renderPasses: RenderPass {
                                        shaderProgram: ShaderProgram {
                                            //vertexShaderCode: loadSource("qrc:/shader/pointcloud.vert")
                                            //fragmentShaderCode: loadSource("qrc:/shader/pointcloud.frag")
                                            vertexShaderCode: loadSource("qrc:/shader/surfel.vert")
                                            fragmentShaderCode: loadSource("qrc:/shader/surfel.frag")
                                        }
                                    }
                                }
                            }
                            parameters: [
                                Parameter { name: "pointSize"; value: pointSizeSlider.value },
                                Parameter { name: "fieldOfView"; value: mainCamera.fieldOfView },
                                Parameter { name: "fieldOfViewVertical"; value: mainCamera.fieldOfView/mainCamera.aspectRatio },
                                Parameter { name: "nearPlane"; value: mainCamera.nearPlane },
                                Parameter { name: "farPlane"; value: mainCamera.farPlane },
                                Parameter { name: "width"; value: scene3d.width },
                                Parameter { name: "height"; value: scene3d.height }
                            ]
                        }
                        components: [ customMesh, materialPoint, meshTransform, layerPoints, picker ]
                    }

                    Q3D.Entity {
                        id: sphereEntity
                        property Layer layerSolid: Layer {
                                id: solidLayer
                            }
                        property ObjectPicker picker: ObjectPicker {
                            onClicked: console.log("Clicked sphere", pick.distance, pick.triangleIndex)
                        }

                        property var meshTransform: Q3D.Transform {
                            id: sphereTransform
                            property real userAngle: 0.0
                            translation: Qt.vector3d(20, 0, 0)
                            rotation: fromAxisAndAngle(Qt.vector3d(0, 1, 0), userAngle)
                        }
                        property var sphereMesh: SphereMesh {
                            radius: 4
                        }
                        property Material materialPhong: PhongMaterial { }
                        components: [ sphereMesh, materialPhong, meshTransform, layerSolid, picker ]
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
        }
    }

    SystemPalette {
        id: palette
    }
}
