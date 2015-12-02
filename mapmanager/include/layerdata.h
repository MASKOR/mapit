#ifndef __LAYERDATA_H
#define __LAYERDATA_H

#include "upns_globals.h"
#include "upns_interface/services.pb.h"
#include "abstractlayerdata.h"
#include <limits>

namespace upns
{
/**
 * TODO: Might be called ObjectData (Note: Do not use word 'Object', but ScanObject, UpnsObject, Entity,
 */

template<typename LayerDataType>
class EntityData : public AbstractEntityData
{
public:

    /**
     * @brief getData
     * @param x1 axis aligned bounding box lower x
     * @param y1 axis aligned bounding box lower y
     * @param z1 axis aligned bounding box lower z
     * @param x2 axis aligned bounding box upper x
     * @param y2 axis aligned bounding box upper y
     * @param z2 axis aligned bounding box upper z
     * @param clipMode underlying datastructure may allow to serve a larger area around the bb without much effort. If clipping is enabled, the module is forced to cut the are at the edges of the given bb.
     * @param lod level of detail. higher values mean more detail. TODO: negative values? -1 is maximum detail?
     * @return layer data for the area in the bb (and slightly around)
     */
    virtual upnsSharedPointer<LayerDataType> getData(upnsReal x1, upnsReal y1, upnsReal z1,upnsReal x2, upnsReal y2, upnsReal z2, bool clipMode, int lod) = 0;

    /**
     * @brief setData deleted layerData for the given region and replaces it with the given data
     * @param x1 axis aligned bounding box lower x
     * @param y1 axis aligned bounding box lower y
     * @param z1 axis aligned bounding box lower z
     * @param x2 axis aligned bounding box upper x
     * @param y2 axis aligned bounding box upper y
     * @param z2 axis aligned bounding box upper z
     * @param data data to set
     * @param lod level of detail. The layertype module might recompute some detaillevels based on the new data and may discard levels.
     * @return 0 on success. Other than zero indicates errors.
     */
    virtual int setData(upnsReal x1, upnsReal y1, upnsReal z1, upnsReal x2, upnsReal y2, upnsReal z2, upnsSharedPointer<LayerDataType> &data, int lod) = 0;

    /**
     * @brief getData get all the layers data.
     * should only be used if the layer is known to contain a light amount of data
     * @param lod
     * @return
     */
    virtual upnsSharedPointer<LayerDataType> getData( int lod) = 0;

    /**
     * @brief setData set data of the complete layer. overwrite everything
     * @param data
     * @param lod
     * @return
     */
    virtual int setData(upnsSharedPointer<LayerDataType> &data, int lod) = 0;

    // TODO: Make the containing map/mapmanager decide, if the object occupies certain space
    virtual int getEntityBoundingBox(upnsReal &x1, upnsReal &y1, upnsReal &z1, upnsReal &x2, upnsReal &y2, upnsReal &z2) = 0;

    // TODO: Nodes, Transforms, dependencies between Layers/Objects

};

}
#endif
