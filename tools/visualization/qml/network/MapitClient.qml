import QtQuick 2.9
import QtWebSockets 1.1

Item {
    id: root
    readonly property string ident: priv.givenIdent
    readonly property string sessionId: priv.givenSessionId
    property alias active: socket.active
    property alias url: socket.url
    property alias status: socket.status
    readonly property MapitMultiviewNetworkState state: MapitMultiviewNetworkState {}
    property MapitMultiviewPeerState ownState: MapitMultiviewPeerState {}
    onActiveChanged: if(active) {
                         sendOwnState()
                     } else {
                         console.log("Disconnect/Server closed connection.")
                         state.peerToPeerState = ({})
                         state.realtimeObjects.clear()
                         //state.visibleEntityInfos.clear()
                         state.repositoryUrl = ""
                         state.checkoutName = ""
                     }

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
        if(typeof ownState.ident === "undefined" || ownState.ident === ""
        || typeof ownState.sessionId === "undefined" || ownState.sessionId === "") {
            // need to connect first
            return
        }
        var numberOfCallbacks = 0;
        var noWait = true
        //count number of object which need id from server
        for(var r1=0 ; r1 < ownState.realtimeObjects.length ; ++r1) {
            ownState.realtimeObjects[r1].peerOwner = root.ident
            if((ownState.realtimeObjects[r1].ident === "") && (ownState.realtimeObjects[r1].ident !== "pending")) {
                numberOfCallbacks++;
                noWait = false
            }
        }
        console.log("DBG: NUMBER of Callbacks: " + numberOfCallbacks)
        // request ident from server an defer futher processing
        for(var r2=0 ; r2 < ownState.realtimeObjects.length ; ++r2) {
            var currentRto = ownState.realtimeObjects[r2]
            if((currentRto.ident === "") && (currentRto.ident !== "pending")) {
                currentRto.ident = "pending"
                getUniqueIdentifier(function(newIdent, data) {
                    ownState.realtimeObjects[data.idx].ident = newIdent
                    console.log("DBG: set ident: " + newIdent + " to " + data.idx + " noc: " + numberOfCallbacks + "-1")
                    numberOfCallbacks--
                    if(numberOfCallbacks == 0) {
                        root._proceedSendingOwnState(ownState)
                    }
                }, {idx: r2})
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
        var visObjs = []
        var copyOfAllObjects = ownState.allVisualInfoModel.slice()
        for(var i2=0 ; i2 < copyOfAllObjects.length/**/ ; ++i2) {
            var orig2 = copyOfAllObjects[i2]
            if(!orig2) continue
            if(!orig2.isVisible) continue
            if(!orig2.isEntity) continue
            // We have to list copied properties here or we transmit all qml properies...
            var visObj = {
                path: orig2.path,
                peerOwner: root.ident,
                additionalData: orig2.additionalData
            }
            visObjs.push(visObj)
        }

        var ownStateJson = { ident: ownState.ident
                           , sessionId: ownState.sessionId
                           , peername: ownState.peername
                           , visibleEntityInfos: visObjs
                           , realtimeObjects: rtos
                           , timestamp: Date.now()
                           , isHost: ownState.isHost
                           , repositoryPort: ownState.repositoryPort
                           , checkoutName: ownState.checkoutName }

        var message = { messagetype: "state", message: ownStateJson }
        socket.sendTextMessage(JSON.stringify(message))
    }

    function getUniqueIdentifier(callback, callbackData) {
        var hashValue = "data_" + (++priv.uidCounter) + Date.now()
        priv.callbackHash[hashValue] = {cb: callback, cbData: callbackData }
        var message = {messagetype: "get_uid", message: { ident: root.ident, sessionId: root.sessionId, data: hashValue }}
        console.log("DBG: Requested IDENT for: " + JSON.stringify(message.message))
        socket.sendTextMessage(JSON.stringify(message))
    }

    readonly property var _priv: QtObject {
        id: priv
        property string givenIdent
        property string givenSessionId
        property var callbackHash: ({})
        property int uidCounter: 234
        onGivenIdentChanged: console.log("DBG: my ident is: " + givenIdent)
    }
    Timer {
        repeat: true
        running: true
        interval: 10
        onTriggered: processMessages()
    }
    function processMessages() {
        var msgCpy = socket.messages.slice()
        socket.messages = []
        //console.log("DBG: 2 len " + msgCpy.length)
        for(var i=0 ; i < msgCpy.length ; ++i) {
            var msg = msgCpy[i]
            var msgData = msg.message
            switch(msg.messagetype) {
            case "connect_reply":
                if(typeof msgData.ident !== "string") {
                    console.log(qsTr("Client received reply with invalid ident: %1").arg(message))
                    return
                }
                priv.givenIdent = msgData.ident
                priv.givenSessionId = msgData.sessionId
                console.log("Connected to mapit server, ownId: " + priv.givenIdent )
                root.sendOwnState();
                break
            case "state_reply":
                if(typeof msgData.status !== "string") {
                    console.log(qsTr("Error while sending state to server: %1").arg(msgData.status))
                }
                break
            case "get_uid_reply":
                priv.callbackHash[msgData.data].cb(msgData.uid, priv.callbackHash[msgData.data].cbData)
                delete priv.callbackHash[msgData.data]
                break
            case "world":

                state.repositoryUrl = msgData.repositoryUrl
                state.checkoutName = msgData.checkoutName
                state.worldUpdated(msgData.world)
                break
            default:
                console.log(qsTr("Server received unknown message: %1").arg(message));
            }
        }
    }
    readonly property WebSocket _webSocket: WebSocket {
        id: socket
        property var messages: ([])
        onTextMessageReceived: {
            console.log("Client received message: " + message)
            var msg = JSON.parse(message)
            socket.messages.push(msg)
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
