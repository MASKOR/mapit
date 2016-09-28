#ifndef ZMQENTITYDATASTREAMPROVIDER_H
#define ZMQENTITYDATASTREAMPROVIDER_H

#include "upns_globals.h"
#include "versioning/repository.h"
#include "zmqnode.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"

namespace upns
{

class ZmqEntitydataStreamProvider : public AbstractEntityDataStreamProvider
{
    // AbstractEntityDataStreamProvider interface
public:
    ZmqEntitydataStreamProvider(upnsString checkoutName, upnsString pathOrOid, ZmqNode *node);
    bool isCached();
    bool isReadWriteSame();
    upnsIStream *startRead(upnsuint64 start, upnsuint64 len);
    void endRead(upnsIStream *strm);
    upnsOStream *startWrite(upnsuint64 start, upnsuint64 len);
    void endWrite(upnsOStream *strm);
    upnsuint64 getStreamSize() const;
    void setStreamSize(upnsuint64);
    LockHandle lock();
    void unlock(LockHandle);
private:
    upnsString m_checkoutName;
    upnsString m_pathOrOid;
    ZmqNode *m_node;
    upnsSharedPointer<upns::Entity> m_e; //< Introduces another roundtrip over network. TODO: Empower ReplyEntitydata to make second roundtrip needless.
    upnsSharedPointer<upns::ReplyEntitydata> m_ed;
    void initHead() const;
    void initEntity() const;
};

}
#endif // ZMQENTITYDATASTREAMPROVIDER_H
