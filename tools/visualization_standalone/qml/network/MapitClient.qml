import QtQuick 2.9
import QtWebSockets 1.1

QtObject {
    id: root
    readonly property string ident: priv.givenIdent
    readonly property string sessionId: priv.givenSessionId
    property alias active: socket.active
    property alias url: socket.url
    property alias status: socket.status
    readonly property MapitMultiviewNetworkState state: MapitMultiviewNetworkState {}
    property MapitMultiviewPeerState ownState: MapitMultiviewPeerState {}
    function sendOwnState(ownState) {
        if(typeof ownState === "undefined") {
            ownState = root.ownState
        }
        if(typeof ownState.sessionId === "undefined" || ownState.sessionId === "") {
            ownState.sessionId = root.sessionId
        }
        if(typeof ownState.ident === "undefined" || ownState.ident === "") {
            ownState.ident = root.ident
        }
        var numberOfCallbacks = 0;
        var noWait = true
        //count number of object which need id from server
        for(var r1=0 ; r1 < ownState.realtimeObjects.length ; ++r1) {
            ownState.realtimeObjects[r1].peerOwner = root.ident
            if(ownState.realtimeObjects[r1].ident === "") {
                numberOfCallbacks++;
                noWait = false
            }
        }
        // request ident from server an defer futher processing
        for(var r2=0 ; r2 < ownState.realtimeObjects.length ; ++r2) {
            var currentRto = ownState.realtimeObjects[r2]
            if(currentRto.ident === "") {
                getUniqueIdentifier(function(newIdent) {
                    currentRto.ident = newIdent
                    numberOfCallbacks--
                    if(numberOfCallbacks == 0) {
                        root._proceedSendingOwnState(ownState)
                    }
                })
            }
        }

        // if all ids where present, proceed without waiting
        if(noWait) _proceedSendingOwnState(ownState)
    }

    // this is used internally for async processing
    function _proceedSendingOwnState(ownState) {
        // JSONify State
        var rtos = []
        for(var i=0 ; i < ownState.realtimeObjects.length ; ++i) {
            var orig = ownState.realtimeObjects[i]
            // We have to list copied properties here or we transmit all qml properies...
            var transf = [orig.tf.m11, orig.tf.m12, orig.tf.m13, orig.tf.m14,
                          orig.tf.m21, orig.tf.m22, orig.tf.m23, orig.tf.m24,
                          orig.tf.m31, orig.tf.m32, orig.tf.m33, orig.tf.m34,
                          orig.tf.m41, orig.tf.m42, orig.tf.m43, orig.tf.m44]
            var rto = {ident: orig.ident,
                peerOwner: orig.peerOwner,
                tf: transf,
                vel: [orig.vel.x, orig.vel.y, orig.vel.z ],
                type: orig.type,
                additionalData: orig.additionalData
            }
//            for(var prop in original) {
//                if(original.hasOwnProperty(prop))
//                    rto[prop] = original[prop]
//            }
            rtos.push(rto)
        }
        var ownStateJson = {ident: ownState.ident, sessionId: ownState.sessionId, peername: ownState.peername, visibleObjects: [], realtimeObjects: rtos, timestamp: Date.now()}

        var message = {messagetype: "state", message: ownStateJson}
//        console.log("DBG: sending own state: " + JSON.stringify(message))
        socket.sendTextMessage(JSON.stringify(message))
    }

    function getUniqueIdentifier(callback) {
        var hashValue = "data_" + (++priv.uidCounter) + Date.now()
        priv.callbackHash[hashValue] = callback
        var message = {messagetype: "get_uid", message: { ident: root.ident, sessionId: root.sessionId, data: hashValue }}
        socket.sendTextMessage(JSON.stringify(message))
    }

    readonly property var _priv: QtObject {
        id: priv
        property string givenIdent
        property string givenSessionId
        property var callbackHash: ({})
        property int uidCounter: 234
    }
    readonly property WebSocket _webSocket: WebSocket {
        id: socket
        onTextMessageReceived: {
            //console.log(qsTr("Client received message: %1").arg(message))
            var msg = JSON.parse(message)
            var msgData = msg.message
            switch(msg.messagetype) {
            case "connect_reply":
                if(typeof msgData.ident !== "string") {
                    console.log(qsTr("Client received reply with invalid ident: %1").arg(message))
                    return
                }
                priv.givenIdent = msgData.ident
                priv.givenSessionId = msgData.sessionId
                break
            case "state_reply":
                if(typeof msgData.status !== "string") {
                    console.log(qsTr("Error while sending state to server: %1").arg(msgData.status))
                }
                break
            case "get_uid_reply":
                priv.callbackHash[msgData.data](msgData.uid)
                delete priv.callbackHash[msgData.data]
                break
            case "world":
                state.worldUpdated(msgData.world)
                break
            default:
                console.log(qsTr("Server received unknown message: %1").arg(message));
            }
        }
        onStatusChanged: {
            if (socket.status == WebSocket.Error) {
                console.log(qsTr("Client error: %1").arg(socket.errorString))
            } else if (socket.status == WebSocket.Closed) {
                console.log(qsTr("Client socket closed."))
            } else if (socket.status == WebSocket.Open) {
                console.log(qsTr("Connection opened."))
                var message = { messagetype:"connect" }
                socket.sendTextMessage(JSON.stringify(message))
            }
        }
    }
}
