import Qt3D.Core 2.0 as Q3D
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0
import QtQml 2.2

import fhac.upns 1.0 as UPNS

Q3D.Entity {
    //property alias transformMat: theMeshTransform.matrix
    property alias currentEntitydata : edrender.entitydata
    //property var currentEntitydata
    property alias currentTransform : currentEntitydataTransform
    property alias currentCheckout : currentEntitydataTransform.checkout
    property var mainCameratmp
    property var scene3dtmp
    property Layer layer
    property alias parametersTmp: surfelTechnique.parameters
    property alias coordinateSystem: edrender.coordinateSystem
    property string currentFrameId
    id: pointcloud

    UPNS.TfTransform {
        id: currentEntitydataTransform
        path: currentEntitydata.path
        targetFrame: pointcloud.currentFrameId
        sourceFrame:  pointcloud.currentCheckout.getEntity(pointcloud.currentEntitydata.path).frameId
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
            matrix: (appStyle.tmpUsePreviewMatrix && pointcloud.currentEntitydata.path === appStyle.tmpCurrentEditEntity) ? appStyle.tmpPreviewMatrix : currentEntitydataTransform.matrix
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
                    parameters: surfelTechnique.parameters
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
