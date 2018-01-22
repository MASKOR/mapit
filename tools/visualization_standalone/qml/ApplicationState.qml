import QtQuick 2.4
import QtQml.Models 2.3

import fhac.upns 1.0 as UPNS

import "network"

Item {
    id: root

//    Note: globalRepository comes from cpp and may be used with command line
//    UPNS.Repository {
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

    UPNS.Checkout {
        id: globalCheckout
        repository: root.currentRepository
        name: appStyle.checkoutName
        onNameChanged: appStyle.checkoutName = name
    }

    UPNS.Entitydata {
        id: globalEntitydata
        checkout: root.currentCheckout
        path: root.currentEntityPath
        onPathChanged: {
            console.log("Path of current entity: " + path)
        }
    }

    UPNS.TfTransform {
        id: globalEntityTransform
        checkout: root.currentCheckout
        path: root.currentEntityPath
        targetFrame: root.currentFrameId
        sourceFrame:  root.currentEntity?root.currentEntity.frameId:""
        mustExist: false
    }

    property string currentRepositoryUrl: globalRepository.url
    onCurrentRepositoryUrlChanged: {
        globalRepository.url = currentRepositoryUrl
    }

    property alias currentCheckoutName: globalCheckout.name
    // Zero or one Repository is open at a time
    readonly property var currentRepository: globalRepository
    // At the moment zero or one checkout is open at a time. In the future entities may be showed without a checkout
    readonly property var currentCheckout: globalCheckout

    // realtime objects of connected peers and more
    property MapitClient mapitClient

    property string currentEntityPath
    onCurrentEntityPathChanged: appStyle.tmpCurrentEditEntity = currentEntityPath

    // One Entity is selected at a time
    readonly property var currentEntity: currentCheckout ? currentCheckout.getEntity(currentEntityPath) : null
    // Entitydata may be loaded only once into the UI (here)! Note that it may freeze the UI for a short time (at the moment).
    // TODO: introduce lazy loading on cpp side
    readonly property var currentEntitydata: globalEntitydata

    // Multiple Entities may be visible at a time
    property var visibleEntityPaths: ListModel {}

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
