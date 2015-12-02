#include "mapservice.h"
#include "upns.h"

namespace upns
{

upnsSharedPointer<upns::Map> MapService::getMap(upns::MapIdentifier mapId)
{
    upnsVec<MapIdentifier> mapIds;
    mapIds.push_back(mapId);
    MapVector maps = getMaps( mapIds );
    assert(maps.size() == 1);
    return maps.at(0);
}

StatusCode MapService::storeMap(upnsSharedPointer<Map> map)
{
    MapVector maps;
    maps.push_back(map);
    MapResultsVector res = storeMaps( maps );
    assert(res.size() == 1);
    return res.at(0).second;
}

StatusCode MapService::removeMap(MapIdentifier mapId)
{
    upnsVec<MapIdentifier> mapIds;
    mapIds.push_back(mapId);
    MapResultsVector res = removeMaps( mapIds );
    assert(res.size() == 1);
    return res.at(0).second;
}

}
