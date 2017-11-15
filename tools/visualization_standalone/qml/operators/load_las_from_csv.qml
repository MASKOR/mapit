import QtQuick 2.4
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.1
import QtQuick.Dialogs 1.2

import ".."

ColumnLayout {
    id: root
    //// in ////
    property bool editable
    property var currentCheckout
    property string currentEntityPath

    function fromParameters(params) {
        fileChooser.filename = params.filename;
        entityChooser.currentEntityPath = params.target;
//        demeanCheckbox.checked = params.demean;
//        normalizeCheckbox.checked = params.normalize;
    }

    //// out ////
    property bool valid: fileChooser.valid && entityChooser.valid
    property var parameters: {
        "filename": fileChooser.filename,
        "target": entityChooser.currentEntityPath,
        //"demean": demeanCheckbox.checked,
        //"normalize": normalizeCheckbox.checked
    }

    //// UI ////
    HelperOpenFile {
        Layout.fillWidth: true
        id: fileChooser
        fileExtension: "pcd"
    }
    HelperTarget {
        Layout.fillWidth: true
        id: entityChooser
        currentEntityPath: root.currentEntityPath
        dialogRoot: root
    }
}
