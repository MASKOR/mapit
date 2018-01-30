import QtQuick 2.9

//QtObject {
Item {
    property string ident: ""
    property string peerOwner

    property matrix4x4 tf
    property vector3d vel

    // "camera"
    property string type

    // can be anything, property bindings will work only for top level properties.
    // e.g. will work for data.points but may not work for data.points.first.x (TODO: test)
    // moreover only json native types should be used, not vector3d or matrix4x4, these types
    // will not map between RealtimeObject <-> JSON correctly.
    property var additionalData: ({})
}
