import QtQuick 2.4

Item {
    id: root
    property var finalTransform
    property real textMax: yText.width
    property real barLen: Math.min(root.width/2-textMax, root.height/2-textMax)
    Item {
        x: barLen+textMax
        y: barLen+textMax
        Rectangle {
            width: 1
            height: barLen*(1.0-Math.abs(finalTransform.column(0).z))
            //height: root.width/2*(1.0-Math.abs(finalTransform.column(0).toVector3d().dotProduct(Qt.vector3d(0.0,0.0,1.0))))
            color: "red"
            transformOrigin: Item.TopLeft
            rotation: Math.atan2(finalTransform.column(0).x, finalTransform.column(0).y)*(180.0/Math.PI) + (finalTransform.column(0).z<0?0:Math.PI)
            Text {
                id: xText
                y: parent.height-textMax/2
                text: "x"
                color: parent.color
                rotation: -parent.rotation
                font.pixelSize: 8
            }
        }
        Rectangle {
            width: 1
            height: barLen*(1.0-Math.abs(finalTransform.column(1).z))
            color: "green"
            transformOrigin: Item.TopLeft
            rotation: Math.atan2(finalTransform.column(1).x, finalTransform.column(1).y)*(180.0/Math.PI) + (finalTransform.column(1).z<0?0:Math.PI)
            Text {
                id: yText
                y: parent.height-textMax/2
                text: "y"
                color: parent.color
                rotation: -parent.rotation
                font.pixelSize: 8
            }
        }
        Rectangle {
            width: 1
            height: barLen*(1.0-Math.abs(finalTransform.column(2).z))
            color: "blue"
            transformOrigin: Item.TopLeft
            rotation: Math.atan2(finalTransform.column(2).x, finalTransform.column(2).y)*(180.0/Math.PI) + (finalTransform.column(2).z<0?0:Math.PI)
            Text {
                y: parent.height-textMax/2
                text: "z"
                color: parent.color
                rotation: -parent.rotation
                font.pixelSize: 8
            }
        }
    }
}
