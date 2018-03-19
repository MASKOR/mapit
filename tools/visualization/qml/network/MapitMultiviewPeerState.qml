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
import QtQml.Models 2.3

Item {
    // identifier given on the first request by the server (secret). Only set for own state
    property string sessionId: ""

    // identifier given on the first request by the server (also for other peers to identify peers)
    property string ident

    // human readable name of the peer, can be freely chosen
    property string peername

    property var additionalData: ({})

    property bool isHost

    // repoUrl: is set by server to the host-client ip + repositoryPort
    property int repositoryPort
    property string workspaceName

    // list of path to objects, the peer currently sees (and works with and wants other to see)
    //property var visibleObjects

    // list of objects that are not part of the collaborative workspace, exculsive to the peer
    //property list<RealtimeObject> realtimeObjects
    property list<RealtimeObject> realtimeObjects

    property var allVisualInfoModel: ([])

    property var timestamp: Date.now()
}
