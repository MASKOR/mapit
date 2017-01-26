#ifndef LAZTYPE_H
#define LAZTYPE_H

#include "entitydata.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include "laszip/laszipper.hpp"
#include "laszip/lasunzipper.hpp"

using namespace upns;

// TODO: Is import case needed? (dynamic linking?)
#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif

// Not a good idea because voxelgridfilter uses pcl smart pointers (boost)
typedef upnsSharedPointer<LASzip> upnsLASzipPtr;

extern "C"
{
MODULE_EXPORT void createEntitydata(upnsSharedPointer<AbstractEntitydata> *out, upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider);
}

class LASzipEntitydata : public Entitydata<LASzip>
{
public:
    static const char* TYPENAME();

    LASzipEntitydata(upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider);

    const char *type() const;
    bool                hasFixedGrid() const;
    bool                canSaveRegions() const;
    upnsLASzipPtr getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                bool clipMode,
                                int lod = 0);
    int                 setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                upnsLASzipPtr &data,
                                int lod = 0);

    upnsLASzipPtr getData(int lod = 0);
    int                 setData(upnsLASzipPtr &data, int lod = 0);

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
    upnsLASzipPtr m_e;
};
#endif
