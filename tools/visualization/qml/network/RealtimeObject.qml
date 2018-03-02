/*******************************************************************************
 *
 * Copyright      2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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
