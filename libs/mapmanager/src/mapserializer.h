#ifndef __MAPSERIALIZER_H
#define __MAPSERIALIZER_H

#include "upns_globals.h"
#include "services.pb.h"
#include "abstractentitydatastreamprovider.h"
#include "entitydata.h"

namespace upns
{

using MapVector = upnsVec< upnsSharedPointer<Map> >;
using LayerVector = upnsVec< upnsSharedPointer<Layer> >;

class MapSerializer
{
public:
    virtual ~MapSerializer() {};



//    virtual upnsVec<MapIdentifier> listMaps() = 0;
//    virtual MapVector getMaps(upnsVec<MapIdentifier> &mapIds) = 0;
//    virtual MapResultsVector storeMaps( MapVector &maps ) = 0;
//    virtual upnsSharedPointer<Map> createMap(upnsString name) = 0;
//    virtual MapResultsVector removeMaps( upnsVec<MapIdentifier> &mapIds ) = 0;

//    virtual LayerVector getLayers(MapIdentifier &mapId) = 0;
//    virtual MapResultsVector storeLayers( LayerVector &layers ) = 0;
//    virtual upnsSharedPointer<Layer> createLayer(MapIdentifier &map, upnsString layerName) = 0;
//    virtual MapResultsVector removeLayers( upnsVec<LayerIdentifier> &layerIds ) = 0;

//    virtual LayerVector getEntity(MapIdentifier &mapId) = 0;
//    virtual MapResultsVector storeLayers( LayerVector &layers ) = 0;
//    virtual upnsSharedPointer<Layer> createLayer(MapIdentifier &map, upnsString layerName) = 0;
//    virtual MapResultsVector removeLayers( upnsVec<LayerIdentifier> &layerIds ) = 0;

    //virtual upnsSharedPointer<Map> receiveNewMap(upnsString name) = 0;

    virtual bool canRead() = 0;
    virtual bool canWrite() = 0;

    virtual upnsSharedPointer<AbstractEntityDataStreamProvider> getStreamProvider(MapIdentifier    mapId,
                                                                                  LayerIdentifier  layerId,
                                                                                  EntityIdentifier entityId) = 0;
};

}
#endif
