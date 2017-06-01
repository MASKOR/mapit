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
    id: pointcloud

    UPNS.EntitydataTransform {
        id: currentEntitydataTransform
        path: currentEntitydata.path + ".tf"
        mustExist: false
    }
    property ObjectPicker picker: ObjectPicker {
        onClicked: console.log("Clicked pcd", pick.distance, pick.triangleIndex)
    }
    property var meshTransform: Q3D.Transform {
            id: theMeshTransform
            matrix: (appStyle.tmpUsePreviewMatrix && pointcloud.currentEntitydata.path === appStyle.tmpCurrentEditEntity) ? appStyle.tmpPreviewMatrix : currentEntitydataTransform.matrix
    }
    property GeometryRenderer customMesh: UPNS.EntitydataRenderer {
        id: edrender
    }
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
                        renderStates: [
                            //PointSize { sizeMode: PointSize.Fixed; value: 5.0 }, // exception when closing application in qt 5.7
                            PointSize { sizeMode: PointSize.Programmable }, //supported since OpenGL 3.2
                            DepthTest { depthFunction: DepthTest.Less }
                            //DepthMask { mask: true }
                        ]
                    }
                },
                Technique {
                    id: pointsTechnique
                    parameters: surfelTechnique.parameters
                    filterKeys: [ FilterKey { name: "primitiveType"; value: "point" },
                                  FilterKey { name: "renderstyle";   value:"points" } ]
                    renderPasses: RenderPass {
                        shaderProgram: ShaderProgram {
                            vertexShaderCode: loadSource("qrc:/shader/pointcloud.vert")
                            fragmentShaderCode: loadSource("qrc:/shader/pointcloud.frag")
                        }
                        renderStates: [
                            //PointSize { sizeMode: PointSize.Fixed; value: 5.0 }, // exception when closing application in qt 5.7
                            PointSize { sizeMode: PointSize.Programmable }, //supported since OpenGL 3.2
                            DepthTest { depthFunction: DepthTest.Less }
                            //DepthMask { mask: true }
                        ]
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
