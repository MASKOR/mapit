#ifndef FS_ENTITYDATASTREAMPROVIDER_H
#define FS_ENTITYDATASTREAMPROVIDER_H

#include <upns/operators/serialization/abstractentitydataprovider.h>

namespace upns
{

class FileSystemEntitydataStreamProvider : public AbstractEntitydataProvider
{
public:
    FileSystemEntitydataStreamProvider(const std::string &filenameRead, const std::string &filenameWrite);
    bool isCached();
    bool isReadWriteSame();
    upnsIStream *startRead(upnsuint64 start, upnsuint64 len);
    void endRead(upnsIStream *strm);
    upnsOStream *startWrite(upnsuint64 start, upnsuint64 len);
    void endWrite(upnsOStream *strm);
    upnsuint64 getStreamSize() const;
    void setStreamSize(upnsuint64 streamSize);
    LockHandle lock();
    void unlock(LockHandle);

    const void *startReadPointer(ReadWriteHandle &handle, upnsuint64 start, upnsuint64 len);
    void endReadPointer(const void *ptr, ReadWriteHandle &handle);
    void *startWritePointer(ReadWriteHandle &handle, upnsuint64 start, upnsuint64 len);
    void endWritePointer(void* ptr, ReadWriteHandle &handle);
    upnsString startReadFile(ReadWriteHandle &handle);
    void endReadFile(ReadWriteHandle &handle);
    upnsString startWriteFile(ReadWriteHandle &handle);
    void endWriteFile(ReadWriteHandle &handle);

    ReadWriteType preferredReadType();
    ReadWriteType preferredWriteType();
private:
    std::string m_filenameRead;
    std::string m_filenameWrite;
};

}

#endif
