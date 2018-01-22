import QtQuick 2.9
import QtWebSockets 1.1

// client server communication:
//
// network protocol:
//  goal: minimal working implementation of shared network state
//  Network messages have the following form
//  { messagetype: "<messagetype>", message: {...} }
//  messagetypes from client to server:
//  "connect" -> { }
//    response: "connect_reply" -> { ident: "<ident>", sessionId: "<secret session id>" }
//  "state" -> MapitMultiviewPeerState
//    response: none or "state_reply" -> { peername: "<peername>", state: "name already in use"}
//  "get_uid" -> {data: "<identifier for response>"}
//    response: "get_uid_reply" -> { uid: "<globally unique identifier>", data: "<identifier for response>"}
//  messagetypes from server to client:
//  (see above *_reply-messages)
//  "world -> { world: [MapitMultiviewPeerStates without the peers own state] }
// application layer:
//  there is a list of MapitMultiviewPeerStates
//  the own state can be send to the server

// The server has no interaction with the application, it only listens for clients
// The application has to connect to the server on its own and is a client like everyone else.

QtObject {
    id: root
    property alias url: webServer.url
    property alias host: webServer.host
    property alias listen: webServer.listen
    property alias accept: webServer.accept
    property alias name: webServer.name
    property alias port: webServer.port
    readonly property bool updatePending: webServer.peersNeedUpdate
    readonly property WebSocketServer _webSocketServer: WebSocketServer {
        id: webServer
        name: "Mapit Viewer"
        listen: false
        accept: false
        onListenChanged: console.log("Server is " + (listen?"":"not ") + "listening.")
        property int uniquePlayerIdCounter: 10
        property int uniqueIdCounter: 1234
        property var peerIdentToSessId: ({})
        property var peerStates: ({})
        property var connectedPeers: ({}) // ident -> date last seen
        property var peerWebSockets: ({})
        property bool peersNeedUpdate: false
        function isMessageValidFromRegisteredPeer(msgData, message) {
            if( typeof msgData.sessionId !== "string" ) {
                console.log(qsTr("Server received message without \"sessionId\": %1").arg(message))
                return false
            } else if( typeof msgData.ident !== "string" ) {
                console.log(qsTr("Server received message without \"ident\": %1").arg(message))
                return false
            } else if(typeof webServer.connectedPeers[msgData.ident] === "undefined") {
                console.log(qsTr("Server received message from unknown peer (not connected): %1").arg(message))
                return false
            } else if(typeof webServer.peerIdentToSessId[msgData.ident] !== "string") {
                console.log(qsTr("Server received message from unknown peer (not connected): %1").arg(message))
            } else if( webServer.peerIdentToSessId[msgData.ident] === msgData.sessionId ) {
                //check if secret sessionId fits to public peer identifier
                return true
            } else {
                console.log(qsTr("Server: Client tried to spoof (wrong ident/sessionId pair): %1").arg(message))
                return false
            }
        }

        onClientConnected: {
            console.log(qsTr("Client Connected!"))
            webSocket.onTextMessageReceived.connect(function(message) {
                //console.log(qsTr("Server received message: %1").arg(message))
                var msg = JSON.parse(message)
                switch(msg.messagetype) {
                case "connect":
                    var id = "plr_id" + (uniquePlayerIdCounter++) // public identification for this client. Known by other clients to identify her although name may changed
                    var sid = "sess_id" + (uniquePlayerIdCounter++) // secret to identify client
                    var replyConnect = { messagetype: "connect_reply", message: { ident: id, sessionId: sid }}
                    webSocket.sendTextMessage(JSON.stringify(replyConnect))
                    webServer.connectedPeers[id] = Date.now()
                    webServer.peerWebSockets[id] = webSocket
                    webServer.peerIdentToSessId[id] = sid
                    var emptyState = {ident: id, sessionId:sid, realtimeObjects: []}
                    webServer.peerStates[id] = emptyState
                    webServer.peersNeedUpdate = true
                    break
                case "state":
                    var msgData = msg.message
                    if( !isMessageValidFromRegisteredPeer(msgData, message) ) return
                    if(typeof msgData.peername !== "undefined") {
                        // name update
                        // check if name is already in use
                        for(var ident in webServer.peerStates) {
                            var state = webServer.peerStates[ident]
                            if(state.ident !== ident) console.log("Server has corrupt data (ident Map)")
                            if(state.ident !== msgData.ident) {
                                if(state.peername === msgData.peername) {
                                    console.log(qsTr("Server received message with duplicate peername: %1").arg(message))
                                    var replyStatus = { messagetype: "state_reply", message: { peername: msgData.peername, status: "name already in use (id other:" + state.ident + ", id requestor:" + msgData.ident + ")" }}
                                    webSocket.sendTextMessage(JSON.stringify(replyStatus))
                                    return
                                }
                            }
                        }
                    }
                    // any other update can be made without checks
                    // TODO: maybe check realtime objects peerOwner for the senders ident
                    webServer.peerStates[msgData.ident] = msgData
                    webServer.peersNeedUpdate = true
                    break
                case "get_uid":
                    var msgDataUid = msg.message
                    if( !isMessageValidFromRegisteredPeer(msgDataUid, message) ) return
                    var replyStatusUid = { messagetype: "get_uid_reply", message: { data: msgDataUid.data, uid: "uid_" + (webServer.uniqueIdCounter++) }}
                    webSocket.sendTextMessage(JSON.stringify(replyStatusUid))
                    break
                default:
                    console.log(qsTr("Server received unknown message: %1").arg(message));
                }
            });
        }
        onErrorStringChanged: {
            console.log(qsTr("Server error: %1").arg(errorString));
        }
        onPeersNeedUpdateChanged: {
            if(peersNeedUpdate) sendDelayTimer.start()
        }
    }
    function sendStateToPeers() {
        if(!webServer.peersNeedUpdate) return
        webServer.peersNeedUpdate = false

        var strippedStates = {}
        // remove ident from world-broadcast (ident == session-id for each peer)
        for(var ident in webServer.peerStates) {
            var state = webServer.peerStates[ident]
            var strippedState = {}
            for(var prop in state) {
                if(prop !== "sessionId") {
                    strippedState[prop] = state[prop]
                }
            }
            strippedStates[ident] = strippedState
        }
        for(var ident2 in webServer.peerWebSockets) {
            var webSocket = webServer.peerWebSockets[ident2]
            if(webSocket.status !== WebSocket.Open) {
                console.log(qsTr("Server noticed disconnect of client %1").arg(webServer.peerStates[ident2].peername));
                delete webServer.peerWebSockets[ident2]
                delete webServer.peerStates[ident2]
                continue
            }
            var strippedStatesWithoutOwn = []
            for(var ident3 in strippedStates) {
                if(ident3 === ident2) continue
                strippedStatesWithoutOwn.push(strippedStates[ident3])
            }
            var messageWorld = { messagetype: "world", message: { world: strippedStatesWithoutOwn} }
            webSocket.sendTextMessage(JSON.stringify(messageWorld))
        }
    }

    readonly property Timer _sendDelayTimer: Timer {
        id: sendDelayTimer
        interval: 1 /* minimum ping */
        running: false
        repeat: false
        onTriggered: root.sendStateToPeers()
    }
//    readonly property Timer _timer: Timer {
//        interval: 1000 /* interval to send updates although nothing changed */
//        running: webServer.accept == true
//        repeat: true
//        onTriggered: root.sendStateToPeers()
//    }
}
