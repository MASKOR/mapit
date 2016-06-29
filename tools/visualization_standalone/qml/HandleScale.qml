import QtQuick 2.4

import Qt3D.Core 2.0 as Q3D
import Qt3D.Render 2.0
import Qt3D.Input 2.0
import Qt3D.Extras 2.0

Q3D.Entity {
    property var currentEntitydataTransform
    id: handle
    property Layer layer: Layer {
            id: gizmoLayer
        }
    property var meshTransform: Q3D.Transform {
            matrix: currentEntitydataTransform.matrix
        }
    property GeometryRenderer gizmoMesh: TODO {

        }
    property Material gizmoMaterial: PhongMaterial { }
    components: [ gizmoMesh, gizmoMaterial, meshTransform, layer ]
}
