#ifndef ZMQENTITYDATASTREAMPROVIDER_H
#define ZMQENTITYDATASTREAMPROVIDER_H

#include <upns/typedefs.h>
#include <upns/versioning/repository.h>
#include "zmqprotobufnode.h"
#include <upns/operators/serialization/abstractentitydataprovider.h>

namespace upns
{

/**
 * @brief ZmqEntitydataStreamProvider sends binary data over network
 */

class ZmqEntitydataStreamProvider : public AbstractEntitydataProvider
{
    // AbstractEntitydataProvider interface
public:
    ZmqEntitydataStreamProvider(std::string checkoutName, std::string pathOrOid, ZmqProtobufNode *node);
    bool isCached();
    bool isReadWriteSame();
    upnsIStream *startRead(upnsuint64 start, upnsuint64 length);
    void endRead(upnsIStream *&strm);
    upnsOStream *startWrite(upnsuint64 start, upnsuint64 len);
    void endWrite(upnsOStream *&strm);
    upnsuint64 getStreamSize() const;
    void setStreamSize(upnsuint64 entitylength);
    LockHandle lock();
    void unlock(LockHandle);

    const void *startReadPointer(ReadWriteHandle &handle, upnsuint64 start, upnsuint64 len);
    void endReadPointer(const void *ptr, ReadWriteHandle &handle);
    void *startWritePointer(ReadWriteHandle &handle, upnsuint64 start, upnsuint64 len);
    void endWritePointer(void *ptr, ReadWriteHandle &handle);
    char *startRead(upnsuint64 start, upnsuint64 length, upnsuint64 &outLength);
    //void endRead();
    void endWrite(const char *memory, const upns::upnsuint64 &length, const upnsuint64 &offset);

    std::string startReadFile(ReadWriteHandle &handle);
    void endReadFile(ReadWriteHandle &handle);
    std::string startWriteFile(ReadWriteHandle &handle);
    void endWriteFile(ReadWriteHandle &handle);

    ReadWriteType preferredReadType();
    ReadWriteType preferredWriteType();
private:
    std::string m_checkoutName;
    std::string m_pathOrOid;
    ZmqProtobufNode *m_node;
    mutable upnsuint64 m_entityLength;
};

}
#endif // ZMQENTITYDATASTREAMPROVIDER_H
