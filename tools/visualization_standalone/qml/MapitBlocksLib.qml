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
        id: blockVoxelgrid
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
                    return {src: target};
                }
                function execute() {
                    console.log("exec: " + target + ", l: " + leafsize)
                    globalOperationScheduler.operatorList.push({moduleName:"voxelgridfilter", parameters:parameters})
                }
            }
        }
    }
}
