/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

import QtQuick 2.0
import QtQuick.Controls 1.1
import fhac.mapit 1.0
import QtQuick.Controls.Styles 1.4

import "."

QuickAccessMenu {
    property var currentWorkspace: globalApplicationState.currentWorkspace
    z: 100
    id: frameIdInput
    height: appStyle.controlHeightInner
    model: currentWorkspace ? currentWorkspace.getFrameIds() : []
    Connections {
        target: currentWorkspace
        onIsBusyExecutingChanged: {
            if(!currentWorkspace.isBusyExecuting)
                frameIdInput.model = frameIdInput.currentWorkspace.getFrameIds()
        }
        oninternalWorkspaceChanged: {
            if(!currentWorkspace.is)
                frameIdInput.model = frameIdInput.currentWorkspace.getFrameIds()
        }
    }
    onCurrentWorkspaceChanged: frameIdInput.model = frameIdInput.currentWorkspace.getFrameIds()
}
