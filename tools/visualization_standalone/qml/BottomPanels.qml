import QtQuick 2.4
import QtQuick.Controls 1.4 as QCtl
import QtQuick.Layouts 1.1
import QtQuick.Window 2.2 as Wnd
import QtGraphicalEffects 1.0
import pcl 1.0
import fhac.upns 1.0 as UPNS
import QtQuick 2.0 as QQ2
import "panes"

Item {
    property alias currentOperator: opPane.currentOperator
    property alias currentCheckout: opPane.currentCheckout
    property alias currentEntityPath: opPane.currentEntityPath
    RowLayout {
        anchors.fill: parent
        OperatorPane {
            id: opPane
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }
}
