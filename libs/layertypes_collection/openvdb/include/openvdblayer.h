#ifndef OPENVDBLAYERTYPE_H
#define OPENVDBLAYERTYPE_H

#include <upns/entitydata.h>
#include <upns/operators/serialization/abstractentitydataprovider.h>

#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>
using namespace upns;

// TODO: Is import case needed? (dynamic linking?)
#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif

// Not a good idea because voxelgridfilter uses pcl smart pointers (boost)
typedef upnsSharedPointer<openvdb::FloatGrid> upnsFloatGridPtr;

extern "C"
{
MODULE_EXPORT void createEntitydata(upnsSharedPointer<AbstractEntitydata> *out, upnsSharedPointer<AbstractEntitydataProvider> streamProvider);
}

class FloatGridEntitydata : public Entitydata<openvdb::FloatGrid>
{
public:
    static const char* TYPENAME();

    FloatGridEntitydata(upnsSharedPointer<AbstractEntitydataProvider> streamProvider);

    const char*         type() const;
    bool                hasFixedGrid() const;
    bool                canSaveRegions() const;
    upnsFloatGridPtr  getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                bool clipMode,
                                int lod = 0);
    int                 setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                upnsFloatGridPtr &data,
                                int lod = 0);

    upnsFloatGridPtr  getData(int lod = 0);
    int                 setData(upnsFloatGridPtr &data, int lod = 0);

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
    upnsSharedPointer<AbstractEntitydataProvider> m_streamProvider;
    upnsFloatGridPtr m_floatGrid;
};
#endif
