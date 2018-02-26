import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Controls.Styles 1.4
import ".."
import "../components"

Item {
    id: root
    property ListModel pipelines
    property bool currentExecutableIsPipeline
    property string currentDetailDialogPath
    property string currentDisplayName
    onPipelinesChanged: priv.reinit()
    Component.onCompleted: priv.reinit()

    function selectItemByName(name) {
        for(var i=0; i<listview.model.count; ++i) {
            var operatorItem = listview.model.get(i)
            if(operatorItem.displayName === name) {
                listview.currentIndex = i
                //priv.selectOpOrPipeline(operatorItem)
                break
            }
        }
    }
    Item {
        id: priv
        property ListModel finalModel: ListModel {}
        property var operatorModel: globalRepository.operators
        function selectOpOrPipeline(objModelItem) {
            var executableName
            if(typeof objModelItem.operator === "undefined" || objModelItem.operator === null) {
                executableName = objModelItem.displayName
                root.currentExecutableIsPipeline = true
            } else {
                executableName = objModelItem.operator.moduleName
                root.currentExecutableIsPipeline = false
            }
            var dialogPath = (root.currentExecutableIsPipeline?"../pipelines/":"../operators/") + executableName
            root.currentDetailDialogPath = dialogPath
            root.currentDisplayName = executableName
        }

        onOperatorModelChanged: priv.reinit()
        function reinit() {
            priv.finalModel.clear();
            for(var i=0; i<root.pipelines.count ; ++i) {
                var pl = root.pipelines.get(i);
                priv.finalModel.append({"displayName": pl.displayName, "operator": null})
            }
            if(!appStyle.showOnlyPipelines) {
                for(var i2=0; i2<priv.operatorModel.length ; ++i2) {
                    var op = priv.operatorModel[i2];
                    priv.finalModel.append({"displayName": op.moduleName, "operator": op})
                }
            }
        }
    }


    ColumnLayout {
        anchors.fill: parent
        RowLayout {
            Layout.fillWidth: true
            StyledButton {
                id: viewListButton
                isIcon: true
                iconSource: "image://material/ic_view_list"
                checkable: true
                checked: appStyle.currentOperatorListView === 0
                enabled: !checked
                onCheckedChanged: {
                    if(checked) viewModuleButton.checked = false
                }
//                Binding {
//                    target: appStyle
//                    property: "currentOperatorListView"
//                    value: viewListButton.checked ? 1 : 0
//                }
            }
            StyledButton {
                id: viewModuleButton
                isIcon: true
                iconSource: "image://material/ic_view_module"
                checkable: true
                checked: appStyle.currentOperatorListView === 1
                enabled: !checked
                onCheckedChanged: {
                    if(checked) viewListButton.checked = false
                }
            }
            StyledButton {
                id: viewOnlyPipelinesButton
                isIcon: true
                iconSource: "image://icon/lightening"
                tooltip: "Show only Workflows"
                checkable: true
                checked: appStyle.showOnlyPipelines
                onCheckedChanged: {
                    appStyle.showOnlyPipelines = checked
                    priv.reinit()
                }
            }
        }
        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            StyledLabel {
                anchors.centerIn: parent
                visible: !globalRepository.isLoaded
                text: "No Repository loaded"
            }
            BusyIndicator {
                anchors.centerIn: parent
                running: globalRepository.isLoadingOperators
            }
            ScrollView {
                visible: viewListButton.checked && globalRepository.isLoaded
                anchors.fill: parent
                ListView {
                    id: listview
                    model: priv.finalModel
                    delegate: Component {
                        RowLayout {
                            height: appStyle.controlHeightOuter
                            Item {
                                id: iconMarginItem
                                property real generatedWidth: appStyle.iconSize//*(16.0/9.0)
                                width: generatedWidth
                                Image {
                                    sourceSize: Qt.size(iconMarginItem.generatedWidth,appStyle.iconSize)
                                    width: iconMarginItem.generatedWidth
                                    height: appStyle.iconSize
                                    source: gridRoot.model.get(index) ? "image://operator/" + gridRoot.model.get(index).displayName : "image://operator"
                                    fillMode: Image.PreserveAspectFit
                                    anchors.verticalCenter: parent.verticalCenter
                                    smooth: false
                                    mipmap: true
                                }
                            }
                            StyledLabel {
                                Layout.fillHeight: true
                                renderType: Text.NativeRendering
                                text: listview.model.get(index) ? listview.model.get(index).displayName : ""
                                verticalAlignment: Text.AlignVCenter
                                tooltip: {
                                    if(!gridRoot.model.get(index)) return "Error"
                                    var objModelItem = gridRoot.model.get(index)
                                    return qsTr("Show details of <i>%1</i><br>").arg(
                                             objModelItem.displayName )
                                            + (objModelItem.operator?(
                                              "<br><b>compiler</b>: " + objModelItem.operator.compiler
                                            + "<br><b>compilerConfig</b>: " + objModelItem.operator.compilerConfig
                                            + "<br><b>date</b>: " + objModelItem.operator.date
                                            + "<br><b>time</b>: " + objModelItem.operator.time
                                            + "<br><b>moduleName</b>: " + objModelItem.operator.moduleName
                                            + "<br><b>description</b>: " + objModelItem.operator.description
                                            + "<br><b>author</b>: " + objModelItem.operator.author
                                            + "<br><b>moduleVersion</b>: " + objModelItem.operator.moduleVersion
                                            + "<br><b>apiVersion</b>: " + objModelItem.operator.apiVersion):"(this is a pipeline)")
                                }
                                property string currentClassName: gridRoot.model.get(index) ? gridRoot.model.get(index).operator ? gridRoot.model.get(index).operator.moduleName ? gridRoot.model.get(index).operator.moduleName : "" : "" : ""
                                //property var myCompo: compo
                                property var myGraph//: graph?graph:null
                                property bool myIsClass: true
                                property bool dragActive: dragArea.drag.active
                                property real dragStartX
                                property real dragStartY
                                onDragActiveChanged: {
                                    forceActiveFocus();
                                    if (dragActive) {
                                        dragStartX = x;
                                        dragStartY = y;
                                        if(currentClassName == "") return
                                        Drag.start();
                                    } else {
                                        Drag.drop();
                                        x = dragStartX;
                                        y = dragStartY;
                                    }
                                }
                                Drag.dragType: Drag.Internal
                                MouseArea {
                                    id: dragArea
                                    anchors.fill: parent
                                    drag.target: parent
                                    onClicked: listview.currentIndex = index
                                }
                            }
                        }
                    }
                    onCurrentIndexChanged: priv.selectOpOrPipeline(model.get(currentIndex))
                    highlightMoveDuration: appStyle.selectionAnimation ? 1000 : 0
                    highlightResizeDuration: appStyle.selectionAnimation ? 1000 : 0
                    highlight: Rectangle { color: appStyle.highlightColor }
                }
            }
            ScrollView {
                visible: viewModuleButton.checked
                horizontalScrollBarPolicy: Qt.ScrollBarAlwaysOff
                verticalScrollBarPolicy: Qt.ScrollBarAlwaysOff
                anchors.fill: parent
                //anchors.bottomMargin: gridRoot.gridMargin
                GridView {
                    property int maximumButtonSize: 85
                    property int gridMargin: 5
                    property int columnCount: Math.ceil(gridRoot.width/(gridMargin*2+maximumButtonSize))
                    property int cellSize: gridRoot.width/columnCount
                    property int buttonSize: cellSize-(gridMargin*2)
                    id: gridRoot
                    clip: true
                    topMargin: gridMargin
                    cellHeight: cellSize
                    cellWidth: cellSize
                    model: priv.finalModel
                    delegate: Button {
                        id: entityButton
                        x: gridRoot.gridMargin
                        width: gridRoot.buttonSize
                        height: gridRoot.buttonSize
                        style: ButtonStyle {
                            background: Rectangle {
                                property var itemDisplayName: gridRoot.model.get(index) ? gridRoot.model.get(index).displayName : ""
                                property bool selected: root.currentDisplayName ? root.currentDisplayName === itemDisplayName : false
                                border.width: selected ? 1 : 0
                                border.color: appStyle.selectionBorderColor
                                color: selected ? appStyle.highlightColor : appStyle.itemBackgroundColor
                            }
                        }
                        tooltip: {
                            if(!gridRoot.model.get(index)) return "Error"
                            var objModelItem = gridRoot.model.get(index)
                            return qsTr("Show details of <i>%1</i><br>").arg(
                                     objModelItem.displayName )
                                    + (objModelItem.op?(
                                      "<br><b>compiler</b>: " + objModelItem.operator.compiler
                                    + "<br><b>compilerConfig</b>: " + objModelItem.operator.compilerConfig
                                    + "<br><b>date</b>: " + objModelItem.operator.date
                                    + "<br><b>time</b>: " + objModelItem.operator.time
                                    + "<br><b>moduleName</b>: " + objModelItem.operator.moduleName
                                    + "<br><b>description</b>: " + objModelItem.operator.description
                                    + "<br><b>author</b>: " + objModelItem.operator.author
                                    + "<br><b>moduleVersion</b>: " + objModelItem.operator.moduleVersion
                                    + "<br><b>apiVersion</b>: " + objModelItem.operator.apiVersion):"(this is a pipeline)")
                        }
                        Column {
                            y: 8
                            anchors.horizontalCenter: parent.horizontalCenter
                            Image {
                                anchors.topMargin: 8
                                source: gridRoot.model.get(index) ? "image://operator/"+gridRoot.model.get(index).displayName : "image://operator"
                                width: gridRoot.buttonSize*0.6
                                height: gridRoot.buttonSize*0.6
                                sourceSize: Qt.size(width, height)
                                fillMode: Image.PreserveAspectFit
                                anchors.horizontalCenter: parent.horizontalCenter
                                smooth: false
                                mipmap: true
                            }
                            Text {
                                width: gridRoot.buttonSize
                                height: 8
                                text: gridRoot.model.get(index) ? gridRoot.model.get(index).displayName : ""
                                anchors.horizontalCenter: parent.horizontalCenter
                                //anchors.bottom: parent.bottom
                                wrapMode: Text.WrapAnywhere
                                horizontalAlignment: Text.AlignHCenter
                                color: Qt.darker("white")
        //                        font.family: editorContent.labelFontFamily
        //                        font.weight: editorContent.labelFontWeight
                                font.pixelSize: 9
                            }
                        }
                        MouseArea {
                            width: gridRoot.buttonSize
                            height: gridRoot.buttonSize
                            onClicked: priv.selectOpOrPipeline(gridRoot.model.get(index))
                        }
                    }
                }
            }
        }
    }
}
