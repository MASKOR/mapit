#ifndef TFLAYER_H
#define TFLAYER_H

#include "entitydata.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"

#include <Eigen/Geometry>

using namespace upns;

// TODO: Is import case needed? (dynmic linking?)
#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif

extern "C"
{
//Note: Not possible in MSVC/Windows. Breaks shared pointers exchange
//MODULE_EXPORT upnsSharedPointer<AbstractEntitydata> createEntitydata(upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider);
MODULE_EXPORT void createEntitydata(upnsSharedPointer<AbstractEntitydata> *out, upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider);
//MODULE_EXPORT void deleteEntitydata(upnsSharedPointer<AbstractEntitydata> streamProvider);
}

typedef Eigen::Affine3f TfMat;
typedef upns::upnsSharedPointer<TfMat> TfMatPtr;

class TfEntitydata : public Entitydata<TfMat>
{
public:

    TfEntitydata(upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider);

    LayerType           layerType() const;
    bool                hasFixedGrid() const;
    bool                canSaveRegions() const;
    TfMatPtr  getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                bool clipMode,
                                int lod = 0);
    int                 setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                TfMatPtr &data,
                                int lod = 0);

    TfMatPtr  getData(int lod = 0);
    int                 setData(TfMatPtr &data, int lod = 0);

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
    TfMatPtr m_tf;

};

#endif
