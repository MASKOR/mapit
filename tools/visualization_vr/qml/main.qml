/*******************************************************************************
 *
 * Copyright      2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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
        offset: Qt.vector3d(-5.0, -1.8, -2.0)
        //offsetOrientation:
        //offset: cameraProps.circlePosition.plus(Qt.vector3d(0, 45 * Math.sin(cameraProps.circleRotation * 2), 0)).plus(cameraProps.tan.times(-2))
    }


    UPNS.Raycast {
        id: teleportRaycast
        viewMatrix: realtimeHead.tf
        projectionMatrix: vrCam.leftCameraLens.projectionMatrix
        viewportSize: Qt.size(_hmd.renderTargetSize.width, _hmd.renderTargetSize.height)
        pointOnPlane: vrCam.offset
        planeNormal: Qt.vector3d(0.0,1.0,0.0)
        worldDirection: tmpTimer.worldDir
    }

    Q3D.Entity {
        property var mesh: TorusMesh {
            radius: 0.5
            minorRadius: 0.05
            rings: 20
            slices: 5
        }
        property var transf: Q3D.Transform {
            id: transform
            translation: teleportRaycast.worldPosition
            Behavior on translation {
                Vector3dAnimation { duration: 100 }
            }
            rotation: fromAxisAndAngle(Qt.vector3d(1.0, 0.0, 0.0), 90)
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

    Timer {
        id: tmpTimer
        running: true
        repeat: true
        interval: 100
        property var worldDir
        onTriggered: {
            // this is a temporary workaround and there are no property change events here
            realtimeHead.tf = vrCam.trackedObjectMatrixTmp(0).inverted()
            realtimeRightHand.tf = vrCam.trackedObjectMatrixTmp(1).inverted()
            realtimeLeftHand.tf = vrCam.trackedObjectMatrixTmp(2).inverted()
            mapitClient.sendOwnState()
            var dir = realtimeRightHand.tf.row(2)
            worldDir = Qt.vector3d(dir.x, dir.y, dir.z)
            console.log("DBG: Raycast: " + worldDir)
            if(vrCam.isTriggerTmp() && !teleportTimer.running) {
                teleportTimer.start();
                vrCam.offset = teleportRaycast.worldPosition
            }
        }
    }
    Timer {
        id: teleportTimer
        repeat: false
        running: false
        interval: 1000
        //onTriggered: running = false
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
        active: false
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
        //visibleEntityPaths: ListModel { ListElement{ path: "fh_aachen/FARO/eloop_filtered-ovdb-ply" }} <- use this to quickly visualize an entity in lack of UI and network
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
