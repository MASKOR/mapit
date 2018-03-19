/*******************************************************************************
 *
 * Copyright 2017-2018 Daniel Bulla	<d.bulla@fh-aachen.de>
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
import QtQml.Models 2.3

import fhac.mapit 1.0 as Mapit

import "qrc:/qml/network"

Item {
    id: root

//    Note: globalRepository comes from cpp and may be used with command line
//    Mapit.Repository {
//        id: globalRepository
//        url: appStyle.repositoryUrl
//        onUrlChanged: appStyle.repositoryUrl = url
//    }

    Component.onCompleted: {
        if(!globalRepositoryExplicitlySpecifiedCommandline) {
            globalRepository.url = appStyle.repositoryUrl
        }
    }

    Connections {
        target: globalRepository
        onUrlChanged: {
            appStyle.repositoryUrl = url
        }
    }

    Mapit.Workspace {
        id: globalWorkspace
        repository: root.currentRepository
        name: appStyle.workspaceName
        onNameChanged: appStyle.workspaceName = name
    }

    Mapit.Entitydata {
        id: globalEntitydata
        checkout: root.currentWorkspace
        path: root.currentEntityPath
        onPathChanged: {
            console.log("Path of current entity: " + path)
        }
    }

    Mapit.TfTransform {
        id: globalEntityTransform
        checkout: root.currentWorkspace
        path: root.currentEntityPath
        targetFrame: root.currentFrameId
        sourceFrame:  root.currentEntity?root.currentEntity.frameId:""
        mustExist: false
    }

    property string currentRepositoryUrl: globalRepository.url
    onCurrentRepositoryUrlChanged: {
        globalRepository.url = currentRepositoryUrl
    }

    property alias currentWorkspaceName: globalWorkspace.name
    // Zero or one Repository is open at a time
    readonly property var currentRepository: globalRepository
    // At the moment zero or one checkout is open at a time. In the future entities may be showed without a checkout
    readonly property var currentWorkspace: globalWorkspace

    // realtime objects of connected peers and more
    property MapitClient mapitClient

    property string currentEntityPath
    onCurrentEntityPathChanged: appStyle.tmpCurrentEditEntity = currentEntityPath

    // One Entity is selected at a time
    readonly property var currentEntity: currentWorkspace ? currentWorkspace.getEntity(currentEntityPath) : null
    // Entitydata may be loaded only once into the UI (here)! Note that it may freeze the UI for a short time (at the moment).
    // TODO: introduce lazy loading on cpp side
    readonly property var currentEntitydata: globalEntitydata

    // Multiple Entities may be visible at a time
    property ListModel visibleEntityModel: ListModel {} // ListModel {}
    // TODO: Bug: There is a bug in Qt3D NodeInstantiator: It will crash when used with ObjectModel. Thus, there is an explicit ListModel at the moment
    property var allVisualInfoModel

    property string currentFrameId
    readonly property var currentEntityTransform: globalEntityTransform

    property string currentDetailDialog
    property bool currentDetailDialogHasExecuteButton

//    function selectOperator(name, props) {
//        appStyle.tmpPrimitiveType = props.type
//        leftPanels.selectOperator( name )
//    }

//    function selectEntity(path) {

//    }
}
