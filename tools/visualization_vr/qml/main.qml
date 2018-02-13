import QtQuick 2.9
import Qt3D.Core 2.0 as Q3D
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0

import QtQuick 2.0 as QQ2
import QtWebSockets 1.1

import vr 2.0

import "qrc:/qml/network"
import "qrc:/qml/scene"

import fhac.upns 1.0 as UPNS

Q3D.Entity {
    id: root
    property string currentFrameId: ""

    components: RenderSettings {
        StereoFrameGraph {
            id: stereoFrameGraph
            leftCamera: vrCam.leftCamera
            rightCamera: vrCam.rightCamera
            hostState: mapitClient.state.hostState
        }
    }
    Item {
        id: cameraProps
        readonly property real cameraRadius: obstaclesRepeater.radius - 50
        readonly property vector3d circlePosition: Qt.vector3d(cameraRadius * Math.cos(circleRotation), 0.0, cameraRadius * Math.sin(circleRotation))
        readonly property vector3d tan: circlePosition.crossProduct(Qt.vector3d(0, 1, 0).normalized())
        property real circleRotation: 0

        QQ2.NumberAnimation {
            target: cameraProps
            property: "circleRotation"
            from: 0; to: Math.PI * 2
            duration: 10000
            loops: QQ2.Animation.Infinite
            running: false
        }
    }
    // Camera
    VrCamera {
        id: vrCam
        offset: Qt.vector3d(80.0,-90.0,0.0)
        //offsetOrientation:
        //offset: cameraProps.circlePosition.plus(Qt.vector3d(0, 45 * Math.sin(cameraProps.circleRotation * 2), 0)).plus(cameraProps.tan.times(-2))
    }

    // Torus obsctacles
    Q3D.NodeInstantiator {
        id: obstaclesRepeater
        model: 40
        readonly property real radius: 3.0
        readonly property real det: 1.0 / model
        delegate: Q3D.Entity {
            property var mesh: TorusMesh {
                radius: 0.5
                minorRadius: 0.05
                rings: 100
                slices: 20
            }
            property var transf: Q3D.Transform {
                id: transform
                readonly property real angle: Math.PI * 2.0 * index * obstaclesRepeater.det
                translation: Qt.vector3d(obstaclesRepeater.radius * Math.cos(transform.angle),
                                         0.0,
                                         obstaclesRepeater.radius * Math.sin(transform.angle))
                rotation: fromAxisAndAngle(Qt.vector3d(0.0, 1.0, 0.0), -transform.angle * 180 / Math.PI)
            }
            property var matr: PhongMaterial {
                diffuse: Qt.rgba(Math.abs(Math.cos(transform.angle)), 204 / 255, 75 / 255, 1)
                specular: "white"
                shininess: 20.0
            }
            components: [
                mesh,
                transf,
                matr,
                stereoFrameGraph.solidLayer
            ]
        }
    }
    Timer {
        running: true
        repeat: true
        interval: 100
        onTriggered: {
            // this is a temporary workaround and there are no property change events here
            realtimeHead.tf = vrCam.trackedObjectMatrixTmp(0).inverted()
            realtimeRightHand.tf = vrCam.trackedObjectMatrixTmp(1).inverted()
            realtimeLeftHand.tf = vrCam.trackedObjectMatrixTmp(2).inverted()
            mapitClient.sendOwnState()
        }
    }
    Q3D.NodeInstantiator {
        id: trackedObjectsRepeater
        // TO DO: a model does not yet work here (array with indices). temporarily using a number
        model: 4 // default: 0 -> head, 1&2 -> controller/hands, 3&4 -> base station
        delegate: Q3D.Entity {
            Timer {
                running: true
                repeat: true
                interval: 1
                onTriggered: {
                    // this is a temporary workaround and there are no property change events here
                    trackedTransform.matrix = vrCam.trackedObjectMatrixTmp(index+1)
                }
            }
            property var mesh:  TorusMesh {
                    radius: 0.1
                    minorRadius: 0.05
                    rings: 100
                    slices: 20
                }
            property var transf: Q3D.Transform {
                id: trackedTransform
            }
            property var matr: PhongMaterial {
                specular: "white"
                ambient: Qt.rgba(1.0*index,1.0*(index-1.0),0.0,1.0)
                shininess: 20.0
            }
            components: [
//                TrackedObjectMesh {
//                    trackedObjectId: index+1
//                },
                mesh,
                transf,
                matr,
                stereoFrameGraph.solidLayer
            ]
        }
    }
    Timer {
        running: activator.activat && (!mapitClient.status !== WebSocket.Open) //Crash!
        //running: !mapitClient.status !== WebSocket.Open
        interval: 1000
        onTriggered: {
            mapitClient.active = true
            activator.activat = false
        }
    }
    Item {
        objectName: "activator"
        id: activator
        property bool activat: false
    }

    MapitClient {
        id: mapitClient
        objectName: "mapitClient"
        url: "ws://127.0.0.1:55511"
        ownState: MapitMultiviewPeerState {
            id: multiviewPeerState
            peername: "VR Bob Main" + (Math.random()*1000).toFixed(0)
            onPeernameChanged: mapitClient.sendOwnState()
            isHost: false
            realtimeObjects: [
                RealtimeObject {
                    id: realtimeHead
                    type: "frustum"
                    additionalData: { "aspect": 20/8, "fov": 110 }
                },
                RealtimeObject {
                    id: realtimeLeftHand
                    type: "hand"
                    additionalData: { }
                },
                RealtimeObject {
                    id: realtimeRightHand
                    type: "hand"
                    additionalData: { }
                }//,
//                RealtimeObject {
//                    id: realtimeStation1
//                    type: "station"
//                    additionalData: { "aspect": 1, "fov": Math.PI }
//                },
//                RealtimeObject {
//                    id: realtimeStation2
//                    type: "station"
//                    additionalData: { }
//                },
//                RealtimeObject {
//                    tf: vrCam.trackedObjectMatrixTmp(3)
//                    type: "station"
//                    additionalData: { }
//                },
//                RealtimeObject {
//                    tf: vrCam.trackedObjectMatrixTmp(4)
//                    type: "hand"
//                    additionalData: { }
//                },
//                RealtimeObject {
//                    tf: vrCam.trackedObjectMatrixTmp(5)
//                    type: "hand"
//                    additionalData: { }
//                }
            ]
        }
    }
    UPNS.Checkout {
        id: globalCheckout
        repository: globalRepository
        name: "testcheckout"
    }

    MapitScene {
        id: mapitScene
        //visibleEntityPaths: ListModel { ListElement{ path: "demomap/FARO/eloop" }}
        currentFrameId: root.currentFrameId
        mapitClient: mapitClient
        camera: vrCam.leftCamera
        currentCheckout: globalCheckout

        gridLayer: stereoFrameGraph.gridLayer
        gizmoLayer: stereoFrameGraph.gizmoLayer
        invisibleLayer: stereoFrameGraph.invisibleLayer
        pointLayer: stereoFrameGraph.pointLayer
        solidLayer: stereoFrameGraph.solidLayer
    }

}
