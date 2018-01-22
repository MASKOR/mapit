import QtQuick 2.9

QtObject {
    property string ident: ""
    property string peerOwner

    property matrix4x4 tf
    property vector3d vel

    // "camera"
    property string type
}
