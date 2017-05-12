import QtQuick 2.5
import Qt.labs.settings 1.0

Item {
    id: root
    objectName: "appStyle"

    property real backgroundLightness: lighness(backgroundColor)
    function lighness(col) {
        var r = col.r
        var g = col.g
        var b = col.b
        var max = Math.max(r, g, b), min = Math.min(r, g, b);
        return (max + min) / 2;
    }
    signal darkLightChanged;

    function unhiglight(col) {
        if(isDark) {
            return Qt.darker(col)
        } else {
            return Qt.lighter(col)
        }
    }

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
    property color itemColor: "#cccccc"
    property color iconHighlightColor: "#26282a"
    property string labelFontFamily: "Open Sans"
    property int labelFontWeight: Font.Normal
    property int labelFontPixelSize: controlHeight*(12.0/25.0)
    property int headerFontWeight: Font.Bold
    property color headerColor: itemBackgroundColor
    property int maximumControlWidth: 200
    property int controlMargin: 4
    property real controlHeight: 25
    property real radius: 2

    property bool tmpUsePreviewMatrix: false
    property matrix4x4 tmpPreviewMatrix
    property string tmpCurrentEditEntity

    property real iconSize: controlHeight > 48
                             ? 48
                             : controlHeight > 32
                               ? 32
                               : controlHeight > 24
                                 ? 24
                                 : controlHeight > 16
                                   ? 16
                                   : controlHeight > 8
                                     ? 8
                                     : 4

    Settings {
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
    }
}
