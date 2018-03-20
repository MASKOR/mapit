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

import QtQuick 2.5
import Qt.labs.settings 1.0

Item {
    id: root
    objectName: "appStyle"
    signal themeChanged
    signal clickedAction
    function emitThemeChanged() {
        themeChanged()
    }

    property bool captureMouse: true
    function emitClickedAction(mouse) {
        clickedAction(mouse)
    }

    property real backgroundLightness: lighness(backgroundColor)
    function lighness(col) {
        var r = col.r
        var g = col.g
        var b = col.b
        var max = Math.max(r, g, b), min = Math.min(r, g, b);
        return (max + min) / 2;
    }
    signal darkLightChanged;

    function unhighlight(col) {
        if(isDark) {
            return Qt.lighter(col)
        } else {
            return Qt.darker(col)
        }
    }

    property string repositoryUrl: "." // default cfg is the current folder
    property string workspaceName: "myworkspace"

    property bool isDark: backgroundLightness < 0.5
    onIsDarkChanged: darkLightChanged()
    property color textColor: "#ffffff"
    property color textColorDisabled: "#a0a1a2"
    property color selectionColor: "#43adee"
    property color selectionBorderColor: "orange"
    property color highlightColor: "#585a5c"
    //property color itemBackgroundColor: "#2e2f30"
    property color backgroundHighlightColor: "#373839"
    property color backgroundColor: "#2e2f30"//"3e3f40"
    property color viewBorderColor: "#000000"
    property color buttonBorderColor: "#838383"
    property color itemBackgroundColor: "#46484a"
    property color itemColor: unhighlight(itemBackgroundColor)
    property color iconHighlightColor: "#26282a"
    property color background3d: "#c0c0c0"
    property string labelFontFamily: "Open Sans"
    property int labelFontWeight: Font.Normal
    //property int labelFontPixelSize: controlHeight*(12.0/25.0)
    property int labelFontPixelSize: controlHeightInner*(20.0/25.0)
    property int headerFontWeight: Font.Bold
    property color headerColor: itemBackgroundColor
    property int maximumControlWidth: 200
    property int controlMargin: 4
    property real controlHeightInner: 16
    property real controlHeightOuter: controlHeightInner + controlMargin*2
    property real controlHeightContainer: controlHeightInner + controlMargin*2
    property real radius: 2
    property bool useRadians: false
    property bool coordinateSystemYPointsUp: false
    property int gridLines: 20
    property real gridSpacing: 1
    property real pointcloudLod: 1
    property real cameraScale: 1
    property bool showGrid: true
    property bool selectionAnimation: true
    property bool showGizmoAlways: false

    // These are currently used to communicate between operators an rest of the ui
    property bool tmpUsePreviewMatrix: false
    property matrix4x4 tmpPreviewMatrix
    property string tmpCurrentEditEntity

    property bool tmpFollowMouse: false
    property bool tmpPlacePrimitive: false
    property string tmpPrimitiveType
    property string tmpPrimitivePayload
    property vector3d tmpMouseIntersect3D: Qt.vector3d(0.0,0.0,0.0)


    property real iconSize: controlHeightOuter >= 48
                             ? 48
                             : controlHeightOuter >= 32
                               ? 32
                               : controlHeightOuter >= 24
                                 ? 24
                                 : controlHeightOuter >= 16
                                   ? 16
                                   : controlHeightOuter >= 8
                                     ? 8
                                     : 4

    property real windowWidth: 1200
    property real windowHeight: 800
    property int currentOperatorListView: 0
    property bool showOnlyPipelines: true
    property real splitViewLeftWidth: 210
    property real splitViewRightWidth: 220

    Settings {
        property alias repositoryUrl: root.repositoryUrl
        property alias workspaceName: root.workspaceName
        property alias textColor: root.textColor
        property alias textColorDisabled: root.textColorDisabled
        property alias selectionColor: root.selectionColor
        property alias selectionBorderColor: root.selectionBorderColor
        property alias highlightColor: root.highlightColor
        //property color itemBackgroundColor: root.itemBackgroundColor
        property alias backgroundHighlightColor: root.backgroundHighlightColor
        property alias backgroundColor: root.backgroundColor
        property alias viewBorderColor: root.viewBorderColor
        property alias buttonBorderColor: root.buttonBorderColor
        property alias itemBackgroundColor: root.itemBackgroundColor
        property alias itemColor: root.itemColor
        property alias iconHighlightColor: root.iconHighlightColor
        property alias labelFontFamily: root.labelFontFamily
        property alias labelFontWeight: root.labelFontWeight
        property alias labelFontPixelSize: root.labelFontPixelSize
        property alias headerFontWeight: root.headerFontWeight
        property alias headerColor: root.headerColor
        property alias maximumControlWidth: root.maximumControlWidth
        property alias controlMargin: root.controlMargin
        property alias controlHeightInner: root.controlHeightInner
//        property alias controlHeightOuter: root.controlHeightOuter
//        property alias controlHeightContainer: root.controlHeightContainer
        property alias useRadians: root.useRadians
        property alias coordinateSystemYPointsUp: root.coordinateSystemYPointsUp
        property alias gridLines: root.gridLines
        property alias gridSpacing: root.gridSpacing
        property alias pointcloudLod: root.pointcloudLod
        property alias cameraScale: root.cameraScale
        property alias showGrid: root.showGrid
        property alias selectionAnimation: root.selectionAnimation
        property alias showGizmoAlways: root.showGizmoAlways


        property alias windowWidth: root.windowWidth
        property alias windowHeight: root.windowHeight
        property alias showOnlyPipelines: root.showOnlyPipelines
        property alias currentOperatorListView: root.currentOperatorListView
        property alias splitViewLeftWidth: root.splitViewLeftWidth
        property alias splitViewRightWidth: root.splitViewRightWidth
    }
    Item {
        id: defaultDarkTheme
        property color textColor: "#ffffff"
        property color textColorDisabled: "#a0a1a2"
        property color selectionColor: "#43adee"
        property color selectionBorderColor: "orange"
        property color highlightColor: "#585a5c"
        //property color itemBackgroundColor: "#2e2f30"
        property color backgroundHighlightColor: "#373839"
        property color backgroundColor: "#2e2f30"//"3e3f40"
        property color viewBorderColor: "#000000"
        property color buttonBorderColor: "#838383"
        property color itemBackgroundColor: "#46484a"
        property color itemColor: root.unhighlight(itemBackgroundColor)
        property color iconHighlightColor: "#26282a"
        property color background3d: "#c0c0c0"
    }
    Item {
        id: defaultSystemTheme
        SystemPalette {
            id: paletteActive
            colorGroup: SystemPalette.Active
        }
        SystemPalette {
            id: paletteDisabled
            colorGroup: SystemPalette.Disabled
        }
        SystemPalette {
            id: paletteInactive
            colorGroup: SystemPalette.Inactive
        }

        property color textColor: paletteInactive.text
        property color textColorDisabled: paletteDisabled.text
        property color selectionColor: paletteActive.base
        property color selectionBorderColor: paletteActive.highlight
        property color highlightColor: paletteActive.highlight
        //property color itemBackgroundColor: "#2e2f30"
        property color backgroundHighlightColor: paletteInactive.highlight
        property color backgroundColor: paletteInactive.base
        property color viewBorderColor: paletteInactive.dark
        property color buttonBorderColor: paletteInactive.dark
        property color itemBackgroundColor: paletteInactive.button
        property color itemColor: paletteInactive.buttonText
        property color iconHighlightColor: paletteActive.alternateBase
        property color background3d: "#c0c0c0"
    }

    function resetDefaultColorsToTheme(theme) {
        appStyle.textColor = theme.textColor
        appStyle.textColorDisabled = theme.textColorDisabled
        appStyle.selectionColor = theme.selectionColor
        appStyle.selectionBorderColor = theme.selectionBorderColor
        appStyle.highlightColor = theme.highlightColor
        //property color itemBackgroundColor: "#2e2f30"
        appStyle.backgroundHighlightColor = theme.backgroundHighlightColor
        appStyle.backgroundColor = theme.backgroundColor
        appStyle.viewBorderColor = theme.viewBorderColor
        appStyle.buttonBorderColor = theme.buttonBorderColor
        appStyle.itemBackgroundColor = theme.itemBackgroundColor
        appStyle.itemColor = theme.itemColor
        appStyle.iconHighlightColor = theme.iconHighlightColor
        appStyle.background3d = theme.background3d
        if(appStyle.backgroundColor === appStyle.selectionColor) {
            appStyle.selectionColor = root.unhighlight(appStyle.selectionColor)
        }

        root.emitThemeChanged();
    }
    function resetDefaultColorsDark() {
        resetDefaultColorsToTheme(defaultDarkTheme)
    }
    function resetDefaultColorsSystem() {
        resetDefaultColorsToTheme(defaultSystemTheme)
    }
}
