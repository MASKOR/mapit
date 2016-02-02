import QtQuick 2.4

Item {
    id: root
    property var finalTransform
    Item{
        x: parent.width/2
        y: parent.height/2
        Rectangle {
            width: 1
            height: root.width/2*(1.0-Math.abs(finalTransform.column(0).z))
            color: "red"
            transformOrigin: Item.TopLeft
            rotation: Math.atan2(finalTransform.column(0).x, finalTransform.column(0).y)*(180.0/Math.PI) + (finalTransform.column(0).z<0?0:Math.PI)
            Text {
                y: parent.height
                text: "x"
                color: parent.color
                rotation: -parent.rotation
            }
        }
        Rectangle {
            width: 1
            height: root.width/2*(1.0-Math.abs(finalTransform.column(1).z))
            color: "green"
            transformOrigin: Item.TopLeft
            rotation: Math.atan2(finalTransform.column(1).x, finalTransform.column(1).y)*(180.0/Math.PI) + (finalTransform.column(1).z<0?0:Math.PI)
            Text {
                y: parent.height
                text: "y"
                color: parent.color
                rotation: -parent.rotation
            }
        }
        Rectangle {
            width: 1
            height: root.width/2*(1.0-Math.abs(finalTransform.column(2).z))
            color: "blue"
            transformOrigin: Item.TopLeft
            rotation: Math.atan2(finalTransform.column(2).x, finalTransform.column(2).y)*(180.0/Math.PI) + (finalTransform.column(2).z<0?0:Math.PI)
            Text {
                y: parent.height
                text: "z"
                color: parent.color
                rotation: -parent.rotation
            }
        }
    }
}
