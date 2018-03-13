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
