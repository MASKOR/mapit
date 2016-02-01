import QtQuick 2.4
import QtQuick.Controls 1.3
import QtQuick.Window 2.2
import QtQuick.Dialogs 1.2

MenuBar {
    property alias fixUpvector: fixedUpvec.checked
    property alias invertYAxis: invertY.checked
    property alias showCenterCross: showCenter.checked
    property alias enableVr: vrModeEnabled.checked
    property alias mirrorEnabled: vrMirrorOff.vrMirrorEnabled
    property alias mirrorDistorsion: vrMirrorDistorsion.checked
    property alias mirrorRightEye: vrMirrorRight.checked
    property alias mirrorLeftEye: vrMirrorLeft.checked
    property bool uiEnabled
    Menu {
        title: qsTr("&File")
        MenuItem {
            text: qsTr("&Open Project")
            onTriggered: openFileDialog.open()
        }
        MenuItem {
            text: qsTr("&Commit")
            onTriggered: saveFileDialog.open()
        }
        MenuItem {
            text: qsTr("Save Project &As")
            onTriggered: saveFileDialog.open()
        }
        MenuItem {
            text: qsTr("E&xit")
            onTriggered: Qt.quit();
        }
    }
    Menu {
        title: qsTr("&Edit")
        MenuItem {
            text: qsTr("&Copy")
            onTriggered: console.log("not yet implemented.");
        }
        MenuItem {
            text: qsTr("&Paste")
            onTriggered: {

            }
        }
    }
    Menu {
        title: qsTr("&View")
//            MenuItem {
//                id: detailHiItem
//                text: qsTr("&Hi Detail")
//                checkable: true
//                checked: true
//                onCheckedChanged: {
//                }
//            }
//            MenuItem {
//                id: detailMidtem
//                text: qsTr("&Mid Detail")
//                checkable: true
//                checked: false
//                onCheckedChanged: {
//                }
//            }
//            MenuItem {
//                id: detailLoItem
//                text: qsTr("&Low Detail")
//                checkable: true
//                checked: false
//                onCheckedChanged: {
//                }
//            }
        MenuSeparator { }
//            MenuItem {
//                id: executeItem
//                text: qsTr("Lower Detail when &moving")
//                enabled: false
//                onTriggered: {
//                }
//            }
        MenuSeparator { }
        MenuItem {
            id: showCenter
            text: qsTr("Show Center &Cross")
            checkable: true
            checked: true
        }
        MenuSeparator { }
//            MenuItem {
//                id: centerFixed
//                text: qsTr("Torso Yaw fixed to view")
//                checkable: true
//                checked: true
//            }
        MenuItem {
            id: fixedUpvec
            text: qsTr("Fixed Upvector")
            checkable: true
            checked: true
        }
        MenuItem {
            id: invertY
            text: qsTr("Invert Y Axis")
            enabled: !fixedUpvec.checked
            checkable: true
            checked: true
        }
        MenuSeparator { }
        MenuItem {
            id: vrModeEnabled
            text: qsTr("Enable VR")
            checkable: true
            checked: true
        }
        Menu {
            id: vrMirror
            title: qsTr("Mirror VR")
            enabled: vrModeEnabled.checked
            ExclusiveGroup {
                id: mirrorGroup
            }
            MenuItem {
                property bool vrMirrorEnabled: !vrMirrorOff.checked
                id: vrMirrorOff
                text: qsTr("None")
                checkable: uiEnabled
                checked: false
                exclusiveGroup: mirrorGroup
            }
            MenuItem {
                id: vrMirrorDistorsion
                text: qsTr("Distorsion")
                checkable: uiEnabled
                checked: true
                exclusiveGroup: mirrorGroup
            }
            MenuItem {
                id: vrMirrorRight
                text: qsTr("Right Eye")
                enabled: false
                checkable: false && uiEnabled // not yet available
                checked: false
                exclusiveGroup: mirrorGroup
            }
            MenuItem {
                id: vrMirrorLeft
                text: qsTr("Left Eye")
                enabled: false
                checkable: false && uiEnabled // not yet available
                checked: false
                exclusiveGroup: mirrorGroup
            }
        }
    }
    Menu {
        title: qsTr("&Window")
        MenuItem {
            text: qsTr("Show &Operations Pane")
            checkable: true
            checked: true
            onCheckedChanged: {
            }
        }
        MenuItem {
            text: qsTr("Show &Maps/Layers Pane")
            checkable: true
            checked: false
            onCheckedChanged: {
            }
        }
    }
}
