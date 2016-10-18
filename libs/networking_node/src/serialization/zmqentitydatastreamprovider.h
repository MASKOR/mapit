#ifndef ZMQENTITYDATASTREAMPROVIDER_H
#define ZMQENTITYDATASTREAMPROVIDER_H

#include "upns_globals.h"
#include "versioning/repository.h"
#include "zmqprotobufnode.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"

namespace upns
{

/**
 * @brief ZmqEntitydataStreamProvider sends binary data over network
 */

class ZmqEntitydataStreamProvider : public AbstractEntityDataStreamProvider
{
    // AbstractEntityDataStreamProvider interface
public:
    ZmqEntitydataStreamProvider(upnsString checkoutName, upnsString pathOrOid, ZmqProtobufNode *node);
    bool isCached();
    bool isReadWriteSame();
    upnsIStream *startRead(upnsuint64 start, upnsuint64 length);
    void endRead(upnsIStream *strm);
    upnsOStream *startWrite(upnsuint64 start, upnsuint64 len);
    void endWrite(upnsOStream *strm);
    upnsuint64 getStreamSize() const;
    void setStreamSize(upnsuint64 entitylength);
    LockHandle lock();
    void unlock(LockHandle);
private:
    upnsString m_checkoutName;
    upnsString m_pathOrOid;
    ZmqProtobufNode *m_node;
    mutable upnsuint64 m_entityLength;
};

}
#endif // ZMQENTITYDATASTREAMPROVIDER_H
