#ifndef __MAPMANAGER_H
#define __MAPMANAGER_H

#include "upns_globals.h"
#include "mapservice.h"
#include "map.h"

namespace upns
{

class MapManager : public MapService
{
    upnsVec<MapIdentifier> listMaps();
    MapVector getMaps(upnsVec<MapIdentifier> &mapIds);
    upnsVec<upnsPair<MapIdentifier, int> > storeMaps( MapVector &maps );
    upnsSharedPointer<Map> createMap(upnsString &name);

    upnsSharedPointer<Map> doOperation(upnsString &config);
};

}
#endif
