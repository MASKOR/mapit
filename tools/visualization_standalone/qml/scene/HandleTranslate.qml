import QtQuick 2.4

import Qt3D.Core 2.0 as Q3D
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0

Q3D.Entity {
    id: handle
    property Layer layer
    property real axisExtend: 1.0
    property real axisRadius: 0.01
    property real axisPlaneExtend: 0.33
    property real arrowLen: 0.1
    property real arrowBottomRadius: 0.05

    property matrix4x4 matrix

    property var meshAxis: CylinderMesh { // points to z axis by default
        length: handle.axisExtend
        radius: handle.axisRadius
    }
    property var meshArrow: ConeMesh {
        length: handle.arrowLen
        bottomRadius: handle.arrowBottomRadius
    }
    property var meshPlane: PlaneMesh {
        width: axisPlaneExtend
        height: axisPlaneExtend
    }

    Q3D.Entity {
        id: axisX
        property Material material: PhongMaterial { diffuse: "red" }
        property var trans: Q3D.Transform {
            rotationZ: -90
            translation: Qt.vector3d(handle.axisExtend * 0.5, 0.0, 0.0)
        }
        property ObjectPicker picker: ObjectPicker {
            hoverEnabled: true
            dragEnabled: true
        }

        components: [ meshAxis, material, trans, handle.layer, picker ]
        Q3D.Entity {
            id: arrowX
            property var trans: Q3D.Transform {
                translation: Qt.vector3d(0.0, handle.axisExtend * 0.5, 0.0)
            }
            components: [ meshArrow, axisX.material, trans, handle.layer, axisX.picker ]
        }
    }
    Q3D.Entity {
        id: axisY
        property Material material: PhongMaterial { diffuse: "green" }
        property var trans: Q3D.Transform {
            translation: Qt.vector3d(0.0, handle.axisExtend * 0.5, 0.0)
        }
        components: [ meshAxis, material, trans, handle.layer ]
        Q3D.Entity {
            id: arrowY
            property var trans: Q3D.Transform {
                translation: Qt.vector3d(0.0, handle.axisExtend * 0.5, 0.0)
            }
            components: [ meshArrow, axisY.material, trans, handle.layer ]
        }
    }
    Q3D.Entity {
        id: axisZ
        property Material material: PhongMaterial { diffuse: "blue" }
        property var trans: Q3D.Transform {
            rotationX: 90
            translation: Qt.vector3d(0.0, 0.0, handle.axisExtend * 0.5)
        }
        components: [ meshAxis, material, trans, handle.layer ]
        Q3D.Entity {
            id: arrowZ
            property var trans: Q3D.Transform {
                translation: Qt.vector3d(0.0, handle.axisExtend * 0.5, 0.0)
            }
            components: [ meshArrow, axisZ.material, trans, handle.layer ]
        }
    }
    Q3D.Entity {
        id: planeX
        property Material material: PhongMaterial { diffuse: Qt.rgba(1.0,0.0,0.0,0.5) }
        property var trans: Q3D.Transform {
            rotationZ: -90
            translation: Qt.vector3d(0.0, handle.axisPlaneExtend * 0.5, handle.axisPlaneExtend * 0.5)
        }
        components: [ meshPlane, material, trans, handle.layer ]
    }
    Q3D.Entity {
        id: planeY
        property Material material: PhongMaterial { diffuse: Qt.rgba(0.0,1.0,0.0,0.5) }
        property var trans: Q3D.Transform {
            translation: Qt.vector3d(handle.axisPlaneExtend * 0.5, 0.0, handle.axisPlaneExtend * 0.5)
        }
        components: [ meshPlane, material, trans, handle.layer ]
    }
    Q3D.Entity {
        id: planeZ
        property Material material: PhongMaterial { diffuse: Qt.rgba(0.0,0.0,1.0,0.5) }
        property var trans: Q3D.Transform {
            rotationX: 90
            translation: Qt.vector3d(handle.axisPlaneExtend * 0.5, handle.axisPlaneExtend * 0.5, 0.0)
        }
        components: [ meshPlane, material, trans, handle.layer ]
    }
}
