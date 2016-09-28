#ifndef UPNSZMQENTITYDATA_H
#define UPNSZMQENTITYDATA_H

#include <string>
#include "versioning/repository.h"
#include "zmqnode.h"

namespace upns {

///
/// \brief The ZmqRequesterCheckout class
/// Implements the basic Checkout Interface and will send requests over network
///

//TODO: This may be removed completely since zmqentitydatastreamprovider does the job

class ZmqEntitydata : public upns::AbstractEntityData
{
private:
    ZmqEntitydata(upnsString checkoutName, upnsString pathOrOid, ZmqNode *node);

    // AbstractEntityData interface
public:
    LayerType layerType() const;
    bool hasFixedGrid() const;
    bool canSaveRegions() const;
    void gridCellAt(upnsReal x, upnsReal y, upnsReal z, upnsReal &x1, upnsReal &y1, upnsReal &z1, upnsReal &x2, upnsReal &y2, upnsReal &z2) const;
    upnsIStream *startReadBytes(upnsuint64 start, upnsuint64 len);
    void endRead(upnsIStream *strm);
    upnsOStream *startWriteBytes(upnsuint64 start, upnsuint64 len);
    void endWrite(upnsOStream *strm);
private:
    upnsString m_checkoutName;
    upnsString m_pathOrOid;
    ZmqNode *m_node;
    mutable upnsSharedPointer<upns::Entity> m_e; //< for convenience only. Introduces another roundtrip over network
    mutable upnsSharedPointer<upns::ReplyEntitydata> m_ed;
    void initHead() const;
    void initEntity() const;
};

}

#endif // UPNSZMQREQUESTER_H
