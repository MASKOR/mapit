/****************************************************************************
**
** Copyright (C) 2015 Klaralvdalens Datakonsult AB (KDAB).
** Contact: https://www.qt.io/licensing/
**
** This file is part of the Qt3D module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

import Qt3D.Core 2.0
import Qt3D.Render 2.0

Viewport {

    property alias leftCamera: leftCameraSelector.camera
    property alias rightCamera: rightCameraSelector.camera
//    property alias window: surfaceSelector.surface

    property alias gridLayer: gridLayer
    property alias gizmoLayer: gizmoLayer
    property alias invisibleLayer: invisibleLayer
    property alias pointLayer: pointLayer
    property alias solidLayer: solidLayer

    property var hostState
    RenderSurfaceSelector {
        id: surfaceSelector
        //surface: _hmd.surface
        externalRenderTargetSize: _hmd.renderTargetSize

        // ColorMask is reset by default
        // By default reset to the default if not specified
        ClearBuffers {
            buffers: ClearBuffers.ColorDepthBuffer
            clearColor: "grey"
            NoDraw {} // We just want to clear the buffers
        }

        // Draw with left eye
        CameraSelector {
            id: leftCameraSelector
            Viewport {
                normalizedRect: Qt.rect(0,0,0.5,1)
                RenderStateSet {
                    renderStates: [
                        DepthTest { depthFunction: DepthTest.Less }
                    ]
                    FrustumCulling {
                        LayerFilter {
                            Layer {
                                id: noGridLayer
                            }
                            Layer {
                                id: gridLayer
                            }
                            layers: true ? gridLayer : noGridLayer
                        }
                        LayerFilter {
                            layers: Layer {
                                id: invisibleLayer
                            }
                            NoDraw {}
                        }
                        LayerFilter {
                            layers: Layer {
                                id: solidLayer
                            }
                        }
                        LayerFilter {
                            layers: Layer {
                                id: pointLayer
                            }
                            TechniqueFilter {
                                id: techniqueFilter
                                function fieldnameToShaderindex(text) {
                                    if(text === "x") return 0;
                                    if(text === "y") return 1;
                                    if(text === "z") return 2;
                                    if(text === "normal_x") return 3;
                                    if(text === "normal_y") return 4;
                                    if(text === "normal_z") return 5;
                                    if(text === "curvature") return 6;
                                    if(text === "rgb") return 7;
                                    if(text === "intensity") return 8;
                                }

                                matchAll: [
                                    FilterKey { name: "primitiveType"; value: "point" },
                                    FilterKey { name: "renderstyle";   value: "points" } //hostState.additionalData.renderStyle }
                                ]
                                parameters: [
                                    Parameter { name: "colorize"; value: hostState.additionalData.shaderVar },//techniqueFilter.fieldnameToShaderindex("rgb") },
                                    Parameter { name: "colorMode"; value: hostState.additionalData.shaderVar2 },
                                    Parameter { name: "pointSize"; value: hostState.additionalData.pointSize ? hostState.additionalData.pointSize : 15},
                                    Parameter { name: "fieldOfView"; value: vrCam.leftCameraLens.fieldOfView },
                                    Parameter { name: "fieldOfViewVertical"; value: vrCam.leftCameraLens.fieldOfView/vrCam.leftCameraLens.aspectRatio },
                                    Parameter { name: "nearPlane"; value: 0.2},//vrCam.leftCameraLens.nearPlane },
                                    Parameter { name: "farPlane"; value: 1000.0},//vrCam.leftCameraLens.farPlane },
                                    Parameter { name: "width"; value: _hmd.renderTargetSize.width },
                                    Parameter { name: "height"; value: _hmd.renderTargetSize.height },
                                    Parameter { name: "lod"; value: 1 },
                                    Parameter { name: "colorscale"; value: 1 },
                                    Parameter { name: "constantSize"; value: false },
                                    Parameter { name: "yPointsUp"; value: true }
                                ]
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
                        LayerFilter {
                            Layer {
                                id: gizmoLayer
                            }
                            layers: gizmoLayer
                        }
                    }
                }
            }
        }

        // Draw with right eye
        CameraSelector {
            id: rightCameraSelector
            Viewport {
                normalizedRect: Qt.rect(0.5,0,0.5,1)
                RenderStateSet {
                    renderStates: [
                        DepthTest { depthFunction: DepthTest.Less }
                    ]
                    FrustumCulling {
//                        ClearBuffers {
//                            buffers : ClearBuffers.ColorDepthBuffer
//                            clearColor: "blue"
//                            NoDraw {}
//                        }
                        LayerFilter {
                            layers: true ? gridLayer : noGridLayer
                        }
                        LayerFilter {
                            layers: invisibleLayer
                            NoDraw {}
                        }
                        LayerFilter {
                            layers: solidLayer
                        }
                        LayerFilter {
                            layers: pointLayer
                            TechniqueFilter {
                                matchAll: [
                                    FilterKey { name: "primitiveType"; value: "point" },
                                    FilterKey { name: "renderstyle";   value: "points" }//hostState.additionalData.renderStyle }
                                ]
                                parameters: [
                                    Parameter { name: "colorize"; value: hostState.additionalData.shaderVar }, //techniqueFilter.fieldnameToShaderindex("rgb") },
                                    Parameter { name: "colorMode"; value: hostState.additionalData.shaderVar2 },
                                    Parameter { name: "pointSize"; value: hostState.additionalData.pointSize },
                                    Parameter { name: "fieldOfView"; value: Math.PI/180*110 }, //vrCam.rightCameraLens.fieldOfView },
                                    Parameter { name: "fieldOfViewVertical"; value: Math.PI/180*100}, //vrCam.rightCameraLens.fieldOfView/vrCam.rightCameraLens.aspectRatio },
                                    Parameter { name: "nearPlane"; value: 0.2 },//vrCam.rightCameraLens.nearPlane },
                                    Parameter { name: "farPlane"; value: 1000.0},//vrCam.rightCameraLens.farPlane },
                                    Parameter { name: "width"; value: 2160*0.5},//_hmd.renderTargetSize.width },
                                    Parameter { name: "height"; value: 1200},//_hmd.renderTargetSize.height },
                                    Parameter { name: "lod"; value: 1 },
                                    Parameter { name: "colorscale"; value: 1 },
                                    Parameter { name: "constantSize"; value: false },
                                    Parameter { name: "yPointsUp"; value: true }
                                ]
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
                        LayerFilter {
                            layers: gizmoLayer
                        }
                    }
                }
            }
        }
    }
}
