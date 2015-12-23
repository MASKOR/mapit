 
WorkerScript.onMessage = function(message) {
    var theMap = message.mapMan.getMap(message.id)
    WorkerScript.sendMessage({ 'id': message.id, 'name':theMap.name })

}
