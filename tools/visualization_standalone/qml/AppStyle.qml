import QtQuick 2.5

Item {
    property color textColor: "#ffffff"
    property color textColorDisabled: "#a0a1a2"
    property color selectionColor: "#43adee"
    property color selectionBorderColor: "orange"
    property color highlightColor: "#585a5c"
    property color backgroundColor: "#2e2f30"
    property color backgroundHighlightColor: "#373839"
    property color viewBorderColor: "#000000"
    property color buttonBorderColor: "#838383"
    property color itemBackgroundColor: "#46484a"
    property color itemColor: "#cccccc"
    property color iconHighlightColor: "#26282a"
    property string labelFontFamily: "Open Sans"
    property int labelFontWeight: Font.Normal
    property int labelFontPixelSize: 12
    property int headerFontWeight: Font.Bold
    property color headerColor: itemBackgroundColor
    property int maximumControlWidth: 200
    property int controlMargin: 4
}
