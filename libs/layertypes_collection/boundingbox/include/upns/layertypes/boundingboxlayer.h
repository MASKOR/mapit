#ifndef BOUNDINGBOXLAYER_H
#define BOUNDINGBOXLAYER_H

#include <upns/entitydata.h>
#include <upns/operators/serialization/abstractentitydataprovider.h>
#include "datastructs.pb.h"

using namespace upns;

// TODO: Is import case needed? (dynamic linking?)
#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif
namespace upns
{
    typedef upnsSharedPointer<upns::Boundingbox> BoundingboxPtr;
}
extern "C"
{
MODULE_EXPORT void createEntitydata(upnsSharedPointer<AbstractEntitydata> *out, upnsSharedPointer<AbstractEntitydataProvider> streamProvider);
}

class BoundingboxEntitydata : public Entitydata<upns::Boundingbox>
{
public:
    static const char* TYPENAME();

    BoundingboxEntitydata(upnsSharedPointer<AbstractEntitydataProvider> streamProvider);

    const char*         type() const;
    bool                hasFixedGrid() const;
    bool                canSaveRegions() const;
    BoundingboxPtr      getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                bool clipMode,
                                int lod = 0);
    int                 setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                BoundingboxPtr &data,
                                int lod = 0);

    BoundingboxPtr      getData(int lod = 0);
    int                 setData(BoundingboxPtr &data, int lod = 0);

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
    BoundingboxPtr m_aabb;  // Axis Aligned Bounding Box
};

#endif // BOUNDINGBOXLAYER_H
