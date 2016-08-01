#ifndef TFLAYER_H
#define TFLAYER_H

#include "entitydata.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"

#include <QMatrix4x4> //TODO: Setup tf2 with dependecies to console_bridge, catkin, boost.

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
//MODULE_EXPORT upnsSharedPointer<AbstractEntityData> createEntitydata(upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider);
MODULE_EXPORT void createEntitydata(upnsSharedPointer<AbstractEntityData> *out, upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider);
//MODULE_EXPORT void deleteEntitydata(upnsSharedPointer<AbstractEntityData> streamProvider);
}

typedef QMatrix4x4 TfMat;
typedef upns::upnsSharedPointer<TfMat> TfMatPtr;

class TfEntitydata : public EntityData<TfMat>
{
public:

    TfEntitydata(upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider);

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

private:
    upnsSharedPointer<AbstractEntityDataStreamProvider> m_streamProvider;
    TfMatPtr m_tf;
};

#endif
