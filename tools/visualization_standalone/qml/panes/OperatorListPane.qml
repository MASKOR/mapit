import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Controls.Styles 1.4
import ".."

Item {
    id: root
    property string currentPipeline
    property ListModel pipelines
    property var currentOperator
    onPipelinesChanged: priv.reinit()
    Component.onCompleted: priv.reinit()

    function selectOperator(name) {
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
            if(typeof objModelItem.operator === "undefined" || objModelItem.operator === null) {
                root.currentOperator = null
                root.currentPipeline = objModelItem.displayName
            } else {
                root.currentPipeline = ""
                root.currentOperator = objModelItem.operator
            }
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
                                MouseArea {
                                    anchors.fill: parent
                                    onClicked: listview.currentIndex = index
                                }
                            }
                        }
                    }
                    onCurrentIndexChanged: priv.selectOpOrPipeline(model.get(currentIndex))
                    highlightMoveDuration: appStyle.selectionAnimation ? 1000 : 0
                    highlightResizeDuration: appStyle.selectionAnimation ? 1000 : 0
                    highlight: Rectangle { color: appStyle.itemBackgroundColor }
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
                                property bool selected: root.currentOperator ? root.currentOperator.moduleName === gridRoot.model.get(index).displayName : false
                                border.width: selected ? 1 : 0
                                border.color: appStyle.selectionBorderColor
                                color: appStyle.itemBackgroundColor
                            }
                        }
                        tooltip: qsTr("Show details of <i>%1</i>.").arg(
                                     gridRoot.model.get(index) ? gridRoot.model.get(index).displayName : "")
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
