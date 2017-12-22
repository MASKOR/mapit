import QtQuick 2.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.1
import "qrc:/qml/theme/";

import "components"

Item {
    Item {
        id: blockEntity
        property string displayName: "MapitEntity"
        property var compo: Component {
            EntityChooser {
                id: blockMapitEntityText
                property var output: ["currentEntityPath"]
                function serialize() {
                    return {src: blockMapitEntityText.src};
                }
                width: 220
            }
        }
    }
    Item {
        property string displayName: "Voxelgrid"
        property string className: "voxelgridfilter"
        property var compo: Component {
            Item {
                property var parameters: {
                    "target": target,
                    "leafsize": leafsize
                }
                property var output: ["target"]
                property var input: ["target", "leafsize"]
                property string target
                property real leafsize
                function serialize() {
                    return {};
                }
                function execute() {
                    console.log("exec: voxelgridfilter " + target + ", l: " + leafsize)
                    globalOperationScheduler.operatorList.push({moduleName:"voxelgridfilter", parameters:parameters})
                }
            }
        }
    }
    Item {
        property string displayName: "Normalestimation"
        property string className: "normalestimation"
        property var compo: Component {
            Item {
                property var parameters: {
                    "target": target,
                    "radius": radius,
                    "k": k
                }
                property var output: ["target"]
                property var input: ["target", "radius", "k"]
                property string target
                property real radius
                property real k
                function serialize() {
                    return {};
                }
                function execute() {
                    console.log("exec: normalestimation " + target + ", r: " + radius + ", k: " + k)
                    globalOperationScheduler.operatorList.push({moduleName:"normalestimation", parameters:parameters})
                }
            }
        }
    }
    Item {
        property string displayName: "Load Pointcloud"
        property string className: "load_pointcloud"
        property var compo: Component {
            Item {
                property var parameters: {
                    "target": target,
                    "filename": filename
                }
                property var output: ["target"]
                property var input: ["target", "filename"]
                property string target
                property string filename
                function serialize() {
                    return {};
                }
                function execute() {
                    console.log("exec: load_pointcloud " + target + ", f: " + file)
                    globalOperationScheduler.operatorList.push({moduleName:"load_pointcloud", parameters:parameters})
                }
            }
        }
    }
}
