#ifndef FS_ENTITYDATASTREAMPROVIDER_H
#define FS_ENTITYDATASTREAMPROVIDER_H

#include "modules/serialization/abstractentitydatastreamprovider.h"

namespace upns
{

class FileSystemEntitydataStreamProvider : public AbstractEntitydataStreamProvider
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
private:
    std::string m_filenameRead;
    std::string m_filenameWrite;
};

}

#endif
