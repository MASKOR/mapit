import QtQuick 2.4

Item {
    id: root
    property bool moveAlongHeadDirection: true // move where head shows or where torso shows?

    property bool fixedUpVector: true

    property var torsoPos: Qt.vector3d(0.0,0.0,0.0)

    property var torsoOrientation: Qt.matrix4x4(
                                       1.0, 0.0, 0.0, 0.0,
                                       0.0, 1.0, 0.0, 0.0,
                                       0.0, 0.0, 1.0, 0.0,
                                       0.0, 0.0, 0.0, 1.0)

    // inverse head matrix
    property var headOrientationInverse: headOrientation.transposed()

    // orientation to yaw and pitch around (always view)
    property var rotationOrientation: headOrientationInverse.times(torsoOrientation)

    // orientation to move along (may be torso or head)
    property var moveOrientation: moveAlongHeadDirection?rotationOrientation:torsoOrientation

    // input
    property var headOrientation
    // output
    property var matrix: Qt.matrix4x4(
                             1.0, 0.0, 0.0, 0.0,
                             0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 1.0)

    function move(side, up, forward) {
        torsoPos = torsoPos.plus(moveOrientation.row(0).toVector3d().times(side))
        torsoPos = torsoPos.plus(moveOrientation.row(1).toVector3d().times(up))
        torsoPos = torsoPos.plus(moveOrientation.row(2).toVector3d().times(forward))
    }

    function rotateYaw(yaw) {
        var yawMat = Qt.matrix4x4(
            Math.cos(yaw),    0.0, -Math.sin(yaw),    0.0,
            0.0,              1.0, 0.0,               0.0,
            Math.sin(yaw),    0.0, Math.cos(yaw),     0.0,
            0.0,              0.0, 0.0,               1.0)
        if(fixedUpVector) {
            torsoOrientation = yawMat.times(torsoOrientation);
        } else {
            torsoOrientation = headOrientation.times(yawMat.times(headOrientationInverse.times(torsoOrientation)))
        }
    }
    function rotatePitch(pitch) {
        if(fixedUpVector) return;
        var pitchMat = Qt.matrix4x4(
           1.0,              0.0, 0.0,               0.0,
           0.0,  Math.cos(pitch),  -Math.sin(pitch), 0.0,
           0.0,  Math.sin(pitch),   Math.cos(pitch), 0.0,
           0.0,              0.0, 0.0              , 1.0)
        torsoOrientation = headOrientation.times(pitchMat.times(headOrientationInverse.times(torsoOrientation)))
    }
    function rotateBank(bank) {
        if(fixedUpVector) return;
        var bankMat = Qt.matrix4x4(
           Math.cos(bank), -Math.sin(bank),  0.0, 0.0,
           Math.sin(bank),  Math.cos(bank),  0.0, 0.0,
           0.0,             0.0,             1.0, 0.0,
           0.0,             0.0,             0.0, 1.0)
        torsoOrientation = headOrientation.times(bankMat.times(headOrientationInverse.times(torsoOrientation)))
    }

    // prohibit banking (and pitch)
    function fixUpvector() {
        // Not yet working
        var side // not needed for read
        var upvec = Qt.vector3d(0.0, 1.0, 0.0); // constrain upvector
        var forward = torsoOrientation.row(2).toVector3d() // keep forward direction

        // orthonormalize
        side = upvec.crossProduct(forward).normalized()
        upvec = forward.crossProduct(side).normalized()
        forward = side.crossProduct(upvec).normalized()
        torsoOrientation.m11 = side.x     ; torsoOrientation.m12 = side.y     ; torsoOrientation.m13 = side.z
        torsoOrientation.m21 = upvec.x    ; torsoOrientation.m22 = upvec.y    ; torsoOrientation.m23 = upvec.z
        torsoOrientation.m31 = forward.x  ; torsoOrientation.m32 = forward.y  ; torsoOrientation.m33 = forward.z
    }
    Item {
        id: privatedata
        property var torsoPositionMatrix: Qt.matrix4x4(
                                 1.0, 0.0, 0.0, root.torsoPos.x,
                                 0.0, 1.0, 0.0, root.torsoPos.y,
                                 0.0, 0.0, 1.0, root.torsoPos.z,
                                 0.0, 0.0, 0.0, 1.0)
    }
    function finishMovement() {
        fixUpvector();
        // todo: mix matrix fields
        matrix = torsoOrientation.times(privatedata.torsoPositionMatrix);
    }
}
