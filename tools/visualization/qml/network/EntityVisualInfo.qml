import QtQuick 2.9

QtObject {
//Item {
    property string path: ""
    property string peerOwner

    // not exchanged over network
    property bool isLoading
    // not exchanged over network
    property bool isVisible: false

    property bool isEntity

    // can be anything, property bindings will work only for top level properties.
    // e.g. will work for data.points but may not work for data.points.first.x (TODO: test)
    // moreover only json native types should be used, not vector3d or matrix4x4, these types
    // will not map between RealtimeObject <-> JSON correctly.
    property var additionalData: ({})
}
