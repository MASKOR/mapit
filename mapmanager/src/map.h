//#ifndef __MAP_H
//#define __MAP_H

//#include "upns_globals.h"
//#include "layer.h"
//#include "layerfilter.h"

//namespace upns
//{

//using MapIdentifier = upnsuint64;



//class Map
//{
//public:
//    upnsString mapName() const;
//    void setMapName(const upnsString &mapName);

//    MapIdentifier mapId() const;

//    /**
//     * @brief setMapId
//     * @param mapId
//     * Note: Changing mapId can confuse serialization. When saving a map with changed mapId,
//     * the old map will be unchanged. A new map will then be created (or an other map is overwritten).
//     */
//    void setMapId(const MapIdentifier &mapId);

//    Layer& layer(LayerFilter filter) const;
//    void addLayer(const Layer &layer);

//    /**
//     * @brief operator == only compares Ids. a.mapId() == b.mapId()
//     * @param rhs
//     * @return There is no method, which checks changes in layers, name, ...
//     */
//    bool operator==(const Map& rhs) const;

//    /**
//     * Append all layers of rhs to the map.
//     */
//    Map& operator+=(const Map& rhs);
//    friend Map operator+(Map lhs,
//                      const Map& rhs);
//    Layer& operator[](LayerFilter& filter);
//protected:
//private:
//    upnsString m_mapName;
//    MapIdentifier m_mapId;
//    upnsVec<Layer> m_layer;
//};

//}
//#endif
