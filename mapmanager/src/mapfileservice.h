#ifndef __MAPFILESERVICE_H
#define __MAPFILESERVICE_H

#include "upns_globals.h"
#include "mapservice.h"
//#include "map.h"

namespace leveldb {
    class DB;
}

namespace upns
{

class MapFileService : public MapService
{
public:
    MapFileService(upnsString databaseFileName);
    ~MapFileService();
    upnsVec<MapIdentifier> listMaps();
    MapVector getMaps(upnsVec<MapIdentifier> &mapIds);
    upnsVec<upnsPair<MapIdentifier, int> > storeMaps( MapVector &maps );
    upnsSharedPointer<Map> createMap(upnsString name);
private:
    leveldb::DB* m_db;
};

}
#endif
