pragma Singleton

import QtQuick 2.4

import fhac.upns 1.0

Item {
    id: globals
    /**
     * Gets a map and loads it from MapManager. It uses caching and is NOT notified, if the map changed.
     * If it is known, that the cached map is not valid anymore, set "reloadCache" to true and MapManager
     * will be asked for a new version.
     */
    function getMap(id, reloadCache) {
        console.assert(id === "" || parseInt(id) !== 0, "tried to retrieve mapId 0");
        if(reloadCache === true || typeof privateMembers.maps[id] === "undefined") {
            var theMap = privateMembers.mapManager.getMap(id)
            privateMembers.maps[id] = theMap
        }
        return privateMembers.maps[id]
    }

    function updateCache(id) {
        getMap(id, true)
    }
    function clearCache() {
        mapIdsModel.idToIndex = {};
        privateMembers.maps = {};
        mapIdsModel.clear();
    }
    function doOperation(opDesc, onFinished, reloadLists) {
        var result = privateMembers.mapManager.doOperation(opDesc) //todo. async
        if(typeof onFinished !== "undefined") {
            onFinished( result )
        }
        var typeofReload = typeof reloadLists
        if(typeofReload === "boolean" && reloadLists === true || typeofReload === "undefined") {
            reload( true );
        }
    }
//    function storeMap(mapObject) {
//        var result = privateMembers.mapManager.
//    }
    /**
     * Updates the list of known mapIds. If "cleanCache" is set to true, getMap() will be forced to get a new version
     * of a map for the next time it is called.
     */
    function reload(cleanCache) {
        if(cleanCache) {
            clearCache();
        }
        var allTheMaps = privateMembers.mapManager.listMaps();
        allTheMaps.forEach(function(id) {
            mapIdsModel.appendMapIfNew(id);
        });
    }

    /**
     * This model can be used from controls to show maps.
     * If a control wants to show more info about a map, it can use getMap(id) to do so.
     */
    property ListModel mapIdsModel: ListModel {
        property var idToIndex // allows to check for duplicates without loop
        Component.onCompleted: {
            idToIndex = {}
            globals.reload(false);
        }
        function appendMapIfNew(id) {
            if(idToIndex[id] !== undefined) {
                return
            }
            console.assert(id !== "" && parseInt(id) !== 0, "tried to append mapId 0");
            idToIndex[id] = count
            append({'mapId':id})
        }
        function indexOfMap(id) {
            console.assert(id !== "" && parseInt(id) !== 0, "tried to retieve index of mapId 0");
            appendMapIfNew(id)
            return idToIndex[id]
        }
    }

    Item {
        id: privateMembers
        property var maps
        Component.onCompleted: {
            maps = {}
        }
        property MapManager mapManager: MapManager {}
    }
    property alias _mapManager : privateMembers.mapManager
}
