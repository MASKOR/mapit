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
import "qrc:/qml/scene"
import QtQuick.Scene3D 2.0
import Qt3D.Core 2.0 as Q3D
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0
import QtQml.Models 2.3

import pcl 1.0

import fhac.upns 1.0 as UPNS

import QtQuick 2.0 as QQ2

import "qrc:/qml/network"

Q3D.Entity {
    id: sceneRoot
    property var visibleEntityPaths
    property var allEntities
    property string currentFrameId: ""
    property var mapitClient
    property var camera
    property alias coordianteSystemTransform: coordianteSystemTransform
    property alias gizmoEntity: gizmoEntity
    property alias entityInstantiator: entityInstantiator
    property var currentCheckout
    property var appStyleOptional

    property Layer gridLayer
    property Layer gizmoLayer
    property Layer invisibleLayer
    property Layer pointLayer
    property Layer solidLayer

    Q3D.Entity {
        id: worldEntity
        components: [
            Q3D.Transform {
                id: worldTransform
                scale: 1
            }]
        Q3D.Entity {
            id: gridEntity
            Q3D.Transform {
                id: gridTransform
                translation: Qt.vector3d(0, 0, 0)
            }
            HelperGridMesh {
                id: gridMesh
                gridSpacing: sceneRoot.appStyleOptional ? sceneRoot.appStyleOptional.gridSpacing : 1
                lines: sceneRoot.appStyleOptional ? sceneRoot.appStyleOptional.gridLines : 20
            }

            property Material materialPhong: PhongMaterial {
                ambient: Qt.rgba(0.0,0.0,0.0,1.0)
            }
            components: [ gridMesh, materialPhong, gridTransform, gridLayer ]
        }

//        Timer {
//            id: debugTempSyncTimerRtos
//            running: true
//            interval: 1000
//            repeat: true
//            property int lastLength: 0
//            property int intermediateLength: 0
//            property int currentLength: 0//sceneRoot.mapitClient ? sceneRoot.mapitClient.state.realtimeObjects.count : 0
//            onTriggered: {
//                currentLength = intermediateLength
//                intermediateLength = lastLength
//                lastLength = sceneRoot.mapitClient.state.realtimeObjects.count
//            }
//        }

        Q3D.NodeInstantiator {
            id: realtimeObjectInstantiator
            model: sceneRoot.mapitClient ? sceneRoot.mapitClient.state.realtimeObjects.count : 0
            //active: debugTempSyncTimerRtos.lastLength === debugTempSyncTimerRtos.currentLength
            //model: debugTempSyncTimerRtos.intermediateLength
//                function recalcBoundingBox() {
//                    boundingBoxRecalculator.start()
//                }
//                property int loadingItems: 0
//                property vector3d boundingboxMin // use recalc. Propertybindings would not reevaluate if model changed
//                property vector3d boundingboxMax // use recalc. Propertybindings would not reevaluate if model changed

            //onModelChanged: recalcBoundingBox()
//                onObjectAdded: recalcBoundingBox()
//                onObjectRemoved: recalcBoundingBox()
            Q3D.Entity {
                id: peerGizmo
                property Material material: PhongMaterial { diffuse: "red" }
                property var headRenderer: SphereMesh { }
                //property var planeMesh: PlaneMesh { }
                //property var cylinderRenderer: CylinderMesh { }
                property var handRenderer: ConeMesh { }
                //property var torusMesh: TorusMesh { minorRadius: 0.3 }
                property var boxRenderer: CuboidMesh { }
                property var theRto: sceneRoot.mapitClient.state.realtimeObjects.get(index)
                property var geomRenderer: theRto ? theRto.type === "frustum" ? frustumEntity.geometryRenderer
                                                                     : theRto.type === "head" ? headRenderer
                                                                                              : theRto.type === "hand" ?
                                                                                                    handRenderer : boxRenderer : boxRenderer
                property var gizmoTransform: Q3D.Transform {
                    matrix: peerGizmo.theRto ? peerGizmo.theRto.tf.inverted() : Qt.matrix4x4()
                }
                components: [ gizmoTransform, geomRenderer, material ]
                property var frustumEntity: HandleFrustum {
                layer: gizmoLayer
                    aspectRatio: (peerGizmo.theRto && peerGizmo.theRto.additionalData) ? peerGizmo.theRto.additionalData.aspect : 1
                    horizontalFov: (peerGizmo.theRto && peerGizmo.theRto.additionalData) ? peerGizmo.theRto.additionalData.fov : 1
//                                        Q3D.Entity {
//                                            property Material material: PhongMaterial { diffuse: "red" }
//                                            components: [text1Transform, solidLayer, material, text]
//                                            ExtrudedTextMesh { // Not a type
//                                                id: text
//                                                text: "Hello World"
//                                                width: 20
//                                                height: 10
//                                            }

//                                            Q3D.Transform {
//                                                id: text1Transform
//                                                translation: Qt.vector3d(0, 2, 0)//manipulationGizmo.gizmoTransform.pos.plus( Qt.vector3d(0, 2, 0) )
//                                            }
//                                            TextLabelEntity { // Error when more than 2 peers
//                                                text: manipulationGizmo.theRto ? root.mapitClient.state.peerToPeerState[manipulationGizmo.theRto.peerOwner].peername : "unknown"
//                                                layer: solidLayer //TO DO: would be awesome if children are rendered, when parent is in "solidLayer" automatically
//                                            }
//                                        }
                }
            }
        }

        Q3D.Entity {
            id: gizmoEntity

            property var meshTransform: Q3D.Transform {
                id: viewCenterTransform
                translation: sceneRoot.camera.viewCenter ? sceneRoot.camera.viewCenter : Qt.vector3d(0,0,0)
                rotation: coordianteSystemTransform.rotation
            }
            PerVertexColorMaterial {
                id: perVertexColorMaterial
            }
            GeometryRenderer {
                id: gizmoRenderer

                function rebuild() {
                    buffer.data = buffer.buildGrid()
                }
                instanceCount: 1
                indexOffset: 0
                firstInstance: 0
                vertexCount: 6
                primitiveType: GeometryRenderer.Lines
                geometry: Geometry {
                    Attribute {
                        id: positionAttribute
                        attributeType: Attribute.VertexAttribute
                        vertexBaseType: Attribute.Float
                        vertexSize: 3
                        byteOffset: 0
                        byteStride: 6 * 4
                        count: 6
                        name: "vertexPosition"//defaultPositionAttributeName()
                        buffer: Buffer {
                            id: bufferPosColor
                            type: Buffer.VertexBuffer
                            function buildGrid() {
                                var vertices = 6;
                                var vertexFloats = 6;
                                var vertexArray = new Float32Array(vertexFloats * vertices);
                                vertexArray[ 0] =-1.0; vertexArray[ 1] = 0.0; vertexArray[ 2] = 0.0
                                vertexArray[ 3] = 1.0; vertexArray[ 4] = 0.0; vertexArray[ 5] = 0.0
                                vertexArray[ 6] = 1.0; vertexArray[ 7] = 0.0; vertexArray[ 8] = 0.0
                                vertexArray[ 9] = 1.0; vertexArray[10] = 0.0; vertexArray[11] = 0.0

                                vertexArray[12] = 0.0; vertexArray[13] =-1.0; vertexArray[14] = 0.0
                                vertexArray[15] = 0.0; vertexArray[16] = 1.0; vertexArray[17] = 0.0
                                vertexArray[18] = 0.0; vertexArray[19] = 1.0; vertexArray[20] = 0.0
                                vertexArray[21] = 0.0; vertexArray[22] = 1.0; vertexArray[23] = 0.0

                                vertexArray[24] = 0.0; vertexArray[25] = 0.0; vertexArray[26] =-1.0
                                vertexArray[27] = 0.0; vertexArray[28] = 0.0; vertexArray[29] = 1.0
                                vertexArray[30] = 0.0; vertexArray[31] = 0.0; vertexArray[32] = 1.0
                                vertexArray[33] = 0.0; vertexArray[34] = 0.0; vertexArray[35] = 1.0
                                return vertexArray
                            }
                            data: buildGrid()
                        }
                    }
                    Attribute {
                        id: colorAttribute
                        attributeType: Attribute.VertexAttribute
                        vertexBaseType: Attribute.Float
                        vertexSize: 3
                        byteOffset: 3 * 4
                        byteStride: 6 * 4
                        count: 6
                        name: "vertexColor"//defaultColorAttributeName()
                        buffer: bufferPosColor
                    }
                }
            }
            property bool isVisible
            property Layer currentLayer:  isVisible || (sceneRoot.appStyleOptional && sceneRoot.appStyleOptional.showGizmoAlways) ? solidLayer : invisibleLayer
            components: [ gizmoRenderer, perVertexColorMaterial, viewCenterTransform, currentLayer ]
        }

        Q3D.Entity {
            id: objectsRoot
            property alias objectsRootTransform: coordianteSystemTransform
            components: [
                Q3D.Transform {
                    id: coordianteSystemTransform
                    rotationX: sceneRoot.appStyleOptional ? sceneRoot.appStyleOptional.coordinateSystemYPointsUp ? 0 : -90 : -90 //Math.PI*0.5
                }]

            //                                    HandleTranslate {
            //                                        id: manipulationGizmo
            //                                        layer: gizmoLayer
            //                                        property var gizmoTransform: Q3D.Transform {
            //                                            translation: Qt.vector3d(2.0,2.0,2.0)
            //                                        }
            //                                        components: [ gizmoTransform, gizmoLayer ]
            //                                    }

            UPNS.PointcloudCoordinatesystem {
                id: coordSys
            }
            Timer {
                id: boundingBoxRecalculator
                interval: 1
                repeat: false
                onTriggered: {
                    var minx= Infinity, miny= Infinity, minz= Infinity
                    , maxx=-Infinity, maxy=-Infinity, maxz=-Infinity
                    for(var i=0; i<entityInstantiator.count; i++) {
                        var item = entityInstantiator.objectAt(i)
                        if(minx > item.min.x) minx = item.min.x
                        if(miny > item.min.y) miny = item.min.y
                        if(minz > item.min.z) minz = item.min.z
                        if(maxx < item.max.x) maxx = item.max.x
                        if(maxy < item.max.y) maxy = item.max.y
                        if(maxz < item.max.z) maxz = item.max.z
                    }
                    entityInstantiator.boundingboxMin = Qt.vector3d(minx, miny, minz)
                    entityInstantiator.boundingboxMax = Qt.vector3d(maxx, maxy, maxz)
                }
            }

            Q3D.NodeInstantiator {
                id: entityInstantiator
                function recalcBoundingBox() {
                    boundingBoxRecalculator.start()
                }
                property int loadingItems: 0
                property vector3d boundingboxMin // use recalc. Propertybindings would not reevaluate if model changed
                property vector3d boundingboxMax // use recalc. Propertybindings would not reevaluate if model changed

                model: sceneRoot.visibleEntityPaths ? sceneRoot.visibleEntityPaths.count : 0
                onModelChanged: recalcBoundingBox()
                onObjectAdded: recalcBoundingBox()
                onObjectRemoved: recalcBoundingBox()
                delegate: MapitEntity {
                    mainCameratmp: sceneRoot.camera
                    coordinateSystem: coordSys
                    currentFrameId: sceneRoot.currentFrameId
                    layer: pointLayer
                    //parametersTmp: techniqueFilter.parameters
                    //Currently only one checkout is supported
                    currentCheckout: sceneRoot.currentCheckout
                    //                                    currentCheckout: UPNS.Checkout {
                    //                                        id: co
                    //                                        repository: globalRepository
                    //                                        name: model.checkoutName
                    //                                        //Component.onCompleted: frameIdChooser.addUniqueFrameIds(co.getFrameIds())
                    //                                    }
                    currentEntitydata: UPNS.Entitydata {
                        checkout: sceneRoot.currentCheckout
                        path: (sceneRoot.visibleEntityPaths && sceneRoot.visibleEntityPaths.get(index)) ? sceneRoot.visibleEntityPaths.get(index).path : ""
                        onIsLoadingChanged: {
//                                                    if(isLoading) {
//                                                        // Busy indicator disabled at the moment
//                                                        entityInstantiator.loadingItems++
//                                                    } else {
//                                                        // Busy indicator disabled at the moment
//                                                        entityInstantiator.loadingItems--
//                                                    }
                            if(sceneRoot.allEntities && sceneRoot.visibleEntityPaths.get(index))
                            {
                                var idxInVisualInfoModel = sceneRoot.visibleEntityPaths.get(index).idxInVisualInfoModel
                                sceneRoot.allEntities[idxInVisualInfoModel].isLoading = isLoading
                            }
                        }
                    }
                    onMinChanged: entityInstantiator.recalcBoundingBox()
                    onMaxChanged: entityInstantiator.recalcBoundingBox()
                    HandleBoundingBox {
                        layer: (typeof boundingBoxButton !== "undefined" && boundingBoxButton.checked) ? gizmoLayer : invisibleLayer
                        min: parent.min
                        max: parent.max
                    }
                }
            }

            Timer {
                id: debugTempSyncTimer
                running: true
                interval: 1000
                repeat: true
                property int lastLength: 0
                property int intermediateLength: 0
                property int currentLength: sceneRoot.mapitClient ? sceneRoot.mapitClient.state.visibleEntityInfosList.count : 0
                onTriggered: {
                    currentLength = intermediateLength
                    intermediateLength = lastLength
                    lastLength = sceneRoot.mapitClient.state.visibleEntityInfosList.count
                }
            }

            Q3D.NodeInstantiator {
                id: peerVisualEntityInstantiator
                asynchronous: false
                active: debugTempSyncTimer.lastLength === debugTempSyncTimer.currentLength
//                function recalcBoundingBox() {
//                    boundingBoxRecalculator.start()
//                }
//                property int loadingItems: 0
//                property vector3d boundingboxMin // use recalc. Propertybindings would not reevaluate if model changed
//                property vector3d boundingboxMax // use recalc. Propertybindings would not reevaluate if model changed

                //onModelChanged: recalcBoundingBox()
//                onObjectAdded: recalcBoundingBox()
//                onObjectRemoved: recalcBoundingBox()
                model: debugTempSyncTimer.intermediateLength // sceneRoot.mapitClient ? sceneRoot.mapitClient.state.visibleEntityInfosList.count : 0

                delegate: MapitEntity {
                    mainCameratmp: sceneRoot.camera
                    coordinateSystem: coordSys
                    currentFrameId: sceneRoot.currentFrameId

                    layer: pointLayer
                    //parametersTmp: techniqueFilter.parameters
                    //Currently only one checkout is supported
                    currentCheckout: sceneRoot.currentCheckout
                    currentEntitydata: UPNS.Entitydata {
                        checkout: sceneRoot.currentCheckout
                        path: sceneRoot.mapitClient.state ? sceneRoot.mapitClient.state.visibleEntityInfosList.get(index) ? sceneRoot.mapitClient.state.visibleEntityInfosList.get(index).path : "" : ""
                    }
//                    onMinChanged: peerVisualEntityInstantiator.recalcBoundingBox()
//                    onMaxChanged: peerVisualEntityInstantiator.recalcBoundingBox()
//                    HandleBoundingBox {
//                        layer: ((typeof boundingBoxButton) !== "undefined") && boundingBoxButton.checked ? gizmoLayer : invisibleLayer
//                        min: parent.min
//                        max: parent.max
//                    }
                }
            }

            Q3D.Entity {
                id: annotationPreviewEntity
                property ObjectPicker picker: ObjectPicker {
                    onClicked: console.log("Clicked sphere", pick.distance, pick.triangleIndex)
                }

                property var meshTransform: Q3D.Transform {
                    matrix: sceneRoot.appStyleOptional ? sceneRoot.appStyleOptional.tmpPreviewMatrix : Qt.matrix4x4()
                }
                property var sphereMesh: SphereMesh { }
                property var planeMesh: PlaneMesh { }
                property var cylinderMesh: CylinderMesh { }
                property var coneMesh: ConeMesh { }
                property var torusMesh: TorusMesh { minorRadius: 0.3 }
                property var cubeMesh: CuboidMesh { }
                property var selectedMesh: !sceneRoot.appStyleOptional ? null
                                         : sceneRoot.appStyleOptional.tmpPrimitiveType === "sphere" ? sphereMesh
                                                                                  : sceneRoot.appStyleOptional.tmpPrimitiveType === "plane" ? planeMesh
                                                                                                                          : sceneRoot.appStyleOptional.tmpPrimitiveType === "cylinder" ? cylinderMesh
                                                                                                                                                                     : sceneRoot.appStyleOptional.tmpPrimitiveType === "cone" ? coneMesh
                                                                                                                                                                                                            : sceneRoot.appStyleOptional.tmpPrimitiveType === "torus" ? torusMesh
                                                                                                                                                                                                                                                    : sceneRoot.appStyleOptional.tmpPrimitiveType === "cube" ? cubeMesh
                                                                                                                                                                                                                                                                                           : sphereMesh

                property var materialPhong: PhongMaterial { }
                property Layer currentLayer: sceneRoot.appStyleOptional ? sceneRoot.appStyleOptional.tmpPlacePrimitive ? solidLayer : invisibleLayer : invisibleLayer
                components: [ selectedMesh, materialPhong, meshTransform, currentLayer, picker ]
            }
        }
    }
}
