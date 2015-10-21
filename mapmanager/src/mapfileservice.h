#ifndef __MAPFILESERVICE_H
#define __MAPFILESERVICE_H

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

};

}
#endif
