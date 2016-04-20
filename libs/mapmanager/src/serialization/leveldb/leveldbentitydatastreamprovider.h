#ifndef LEVELDBENTITYDATASTREAMPROVICER_H
#define LEVELDBENTITYDATASTREAMPROVICER_H

#include "modules/serialization/abstractentitydatastreamprovider.h"
#include "leveldb/db.h"

namespace upns
{

class LevelDBEntityDataStreamProvider : public AbstractEntityDataStreamProvider
{
public:
    LevelDBEntityDataStreamProvider(leveldb::DB *db, const std::string &readkey, const std::string &writekey);
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
    leveldb::DB *m_db;
    std::string m_readkey;
    std::string m_writekey;
};

}

#endif
