/*******************************************************************************
 *
 * Copyright 2017-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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

import Qt3D.Core 2.0 as Q3D
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0
import QtQml 2.2

import fhac.upns 1.0 as UPNS

Q3D.Entity {
    id: pointcloud
    //property alias transformMat: theMeshTransform.matrix
    property alias currentEntitydata : edrender.entitydata
    //property var currentEntitydata
    property alias currentTransform : currentEntitydataTransform
    property alias currentCheckout : currentEntitydataTransform.checkout
    property var mainCameratmp
    property Layer layer
    property alias parametersTmp: surfelTechnique.parameters
    property alias coordinateSystem: edrender.coordinateSystem
    property string currentFrameId

    property vector3d min: Qt.vector3d(priv.min.x, priv.min.y, priv.min.z)
    property vector3d max: Qt.vector3d(priv.max.x, priv.max.y, priv.max.z)
    QtObject {
        id: priv
        property bool evalBBox: (pointcloud.currentEntitydata && pointcloud.currentEntitydata.info["min"]) ? true : false
        property bool withTransform: pointcloud.currentTransform.exists ? true : false
        property vector4d minUntf: evalBBox ? Qt.vector4d(pointcloud.currentEntitydata.info["min"].x,
                                                          pointcloud.currentEntitydata.info["min"].y,
                                                          pointcloud.currentEntitydata.info["min"].z, 1.0) : Qt.vector4d(0.0, 0.0, 0.0, 0.0)
        property vector4d maxUntf: evalBBox ? Qt.vector4d(pointcloud.currentEntitydata.info["max"].x,
                                                          pointcloud.currentEntitydata.info["max"].y,
                                                          pointcloud.currentEntitydata.info["max"].z, 1.0) : Qt.vector4d(0.0, 0.0, 0.0, 0.0)
        property vector4d min: withTransform ? pointcloud.currentTransform.matrix.times(minUntf) : minUntf
        property vector4d max: withTransform ? pointcloud.currentTransform.matrix.times(maxUntf) : maxUntf
    }


    UPNS.TfTransform {
        id: currentEntitydataTransform
        path: currentEntitydata.path
        targetFrame: pointcloud.currentFrameId
        sourceFrame:  pointcloud.currentCheckout.getEntity(pointcloud.currentEntitydata.path).frameId
        stamp: pointcloud.currentCheckout.getEntity(pointcloud.currentEntitydata.path).stamp
        mustExist: false

    }
    property ObjectPicker picker: ObjectPicker {
        hoverEnabled: true
        onClicked: {
            if(pick.button == Qt.LeftButton) {
                var transfVec4 = objectsRoot.objectsRootTransform.matrix.inverted().times(Qt.vector4d(pick.worldIntersection.x,pick.worldIntersection.y,pick.worldIntersection.z, 1.0))
                var translateMatrix = Qt.matrix4x4()
                translateMatrix.translate(transfVec4.x, transfVec4.y, transfVec4.z)
                appStyle.tmpPreviewMatrix = translateMatrix
            }
        }
//        onMoved: {
//            var transfVec4 = objectsRoot.objectsRootTransform.matrix.inverted().times(Qt.vector4d(pick.worldIntersection.x,pick.worldIntersection.y,pick.worldIntersection.z, 1.0))
//            var translateMatrix = Qt.matrix4x4()
//            translateMatrix.translate(transfVec4.x, transfVec4.y, transfVec4.z)
//            appStyle.tmpPreviewMatrix = translateMatrix
//            // Currently Scene3D does not work for hover
//            console.log("DBG: Moving mouse (hover) works now " + pick.worldIntersection)
//        }
    }
    property var meshTransform: Q3D.Transform {
            id: theMeshTransform
            matrix: ((typeof appStyle !== "undefined") && appStyle.tmpUsePreviewMatrix && pointcloud.currentEntitydata.path === appStyle.tmpCurrentEditEntity) ? appStyle.tmpPreviewMatrix : currentEntitydataTransform.matrix
    }
    property GeometryRenderer customMesh: UPNS.EntitydataRenderer {
        id: edrender
    }
    property list<RenderState> pointRenderStates: [
        //PointSize { sizeMode: PointSize.Fixed; value: 5.0 }, // exception when closing application in qt 5.7
        PointSize { sizeMode: PointSize.Programmable }, //supported since OpenGL 3.2
        DepthTest { depthFunction: DepthTest.Less }
        //DepthMask { mask: true }
    ]
    property Material materialPoint: Material {
        effect: Effect {
            techniques: [
                Technique {
                    id: surfelTechnique
                    filterKeys: [ FilterKey { name: "primitiveType"; value: "point" },
                                  FilterKey { name: "renderstyle";   value:"surfel" } ]
                    renderPasses: RenderPass {
                        shaderProgram: ShaderProgram {
                            vertexShaderCode: loadSource("qrc:/shader/surfel.vert")
                            fragmentShaderCode: loadSource("qrc:/shader/surfel.frag")
                        }
                        renderStates: pointcloud.pointRenderStates
                    }
                },
                Technique {
                    //parameters: surfelTechnique.parameters
                    filterKeys: [ FilterKey { name: "primitiveType"; value: "point" },
                                  FilterKey { name: "renderstyle";   value:"points" } ]
                    renderPasses: RenderPass {
                        shaderProgram: ShaderProgram {
                            vertexShaderCode: loadSource("qrc:/shader/pointcloud.vert")
                            fragmentShaderCode: loadSource("qrc:/shader/pointcloud.frag")
                        }
                        renderStates: pointcloud.pointRenderStates
                    }
                }
            ]
        }
//        parameters: [
//            Parameter { name: "pointSize"; value: 2.5 },
//            Parameter { name: "fieldOfView"; value: mainCameratmp.fieldOfView },
//            Parameter { name: "fieldOfViewVertical"; value: mainCameratmp.fieldOfView/mainCameratmp.aspectRatio },
//            Parameter { name: "nearPlane"; value: mainCameratmp.nearPlane },
//            Parameter { name: "farPlane"; value: mainCameratmp.farPlane },
//            Parameter { name: "width"; value: scene3dtmp.width },
//            Parameter { name: "height"; value: scene3dtmp.height }
//        ]
    }
    components: [ customMesh, materialPoint, meshTransform, layer, picker ]
}
