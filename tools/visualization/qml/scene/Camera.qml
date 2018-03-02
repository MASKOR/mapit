/*******************************************************************************
 *
 * Copyright 2016-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

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

//    function rotateYaw(yaw) {
//        var yawMat = Qt.matrix4x4(
//            Math.cos(yaw),    0.0, -Math.sin(yaw),    0.0,
//            0.0,              1.0, 0.0,               0.0,
//            Math.sin(yaw),    0.0, Math.cos(yaw),     0.0,
//            0.0,              0.0, 0.0,               1.0)
//        if(fixedUpVector) {
//            torsoOrientation = yawMat.times(torsoOrientation);
//        } else {
//            torsoOrientation = headOrientation.times(yawMat.times(headOrientationInverse.times(torsoOrientation)))
//        }
//    }
//    function rotatePitch(pitch) {
//        if(fixedUpVector) return;
//        var pitchMat = Qt.matrix4x4(
//           1.0,              0.0, 0.0,               0.0,
//           0.0,  Math.cos(pitch),  -Math.sin(pitch), 0.0,
//           0.0,  Math.sin(pitch),   Math.cos(pitch), 0.0,
//           0.0,              0.0, 0.0              , 1.0)
//        torsoOrientation = headOrientation.times(pitchMat.times(headOrientationInverse.times(torsoOrientation)))
//    }
//    function rotateBank(bank) {
//        if(fixedUpVector) return;
//        var bankMat = Qt.matrix4x4(
//           Math.cos(bank), -Math.sin(bank),  0.0, 0.0,
//           Math.sin(bank),  Math.cos(bank),  0.0, 0.0,
//           0.0,             0.0,             1.0, 0.0,
//           0.0,             0.0,             0.0, 1.0)
//        torsoOrientation = headOrientation.times(bankMat.times(headOrientationInverse.times(torsoOrientation)))
//    }
    property var gimbalLockBorder: 0.01
    property var gimbalLockCos: Qt.vector3d(0.0,1.0,0.0).dotProduct(Qt.vector3d(0.0,1.0-gimbalLockBorder,0.0))

    // prohibit banking (and pitch)
    function fixUpvector(dir) {
        // Not yet working
        var side // not needed for read
        var upvec = Qt.vector3d(0.0, 1.0, 0.0); // constrain upvector
        var forward = torsoOrientation.row(2).toVector3d() // keep forward direction
        var dotUpForw = upvec.dotProduct(forward)
        if(Math.abs(dotUpForw) > gimbalLockCos) {
            // Gimbal Lock imminent
            // Note: It would be nice, if we had a direction here.
            // Because this is not the case, the forward vector must be "beamed" back.
            side = torsoOrientation.row(0).toVector3d()
            var minimalForw = upvec.times(-1).crossProduct(torsoOrientation.row(0).toVector3d()).normalized()
            forward = Qt.vector3d(0.0, dotUpForw<0?-1:1, 0.0).plus(minimalForw.times(gimbalLockBorder*15));
        }
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
