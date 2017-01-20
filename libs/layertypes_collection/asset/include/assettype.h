#ifndef ASSETTYPE_H
#define ASSETTYPE_H

#include "entitydata.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include "tinyply.h"

using namespace upns;

// TODO: Is import case needed? (dynamic linking?)
#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif

// Not a good idea because voxelgridfilter uses pcl smart pointers (boost)
typedef upnsSharedPointer<tinyply::PlyFile> upnsAssetPtr;

extern "C"
{
MODULE_EXPORT void createEntitydata(upnsSharedPointer<AbstractEntitydata> *out, upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider);
}

class AssetEntitydata : public Entitydata<tinyply::PlyFile>
{
public:

    AssetEntitydata(upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider);

    LayerType           layerType() const;
    bool                hasFixedGrid() const;
    bool                canSaveRegions() const;
    upnsAssetPtr  getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                bool clipMode,
                                int lod = 0);
    int                 setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                upnsAssetPtr &data,
                                int lod = 0);

    upnsAssetPtr  getData(int lod = 0);
    int                 setData(upnsAssetPtr &data, int lod = 0);

    void gridCellAt(upnsReal   x, upnsReal   y, upnsReal   z,
                    upnsReal &x1, upnsReal &y1, upnsReal &z1,
                    upnsReal &x2, upnsReal &y2, upnsReal &z2) const;

    int getEntityBoundingBox(upnsReal &x1, upnsReal &y1, upnsReal &z1,
                             upnsReal &x2, upnsReal &y2, upnsReal &z2);

    upnsIStream *startReadBytes(upnsuint64 start, upnsuint64 len);
    void endRead(upnsIStream *strm);

    upnsOStream *startWriteBytes(upnsuint64 start, upnsuint64 len);
    void endWrite(upnsOStream *strm);

    size_t size() const;
private:
    upnsSharedPointer<AbstractEntitydataStreamProvider> m_streamProvider;
    upnsAssetPtr m_asset;
};
#endif
