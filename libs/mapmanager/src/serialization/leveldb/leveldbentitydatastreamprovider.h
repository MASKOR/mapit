#ifndef LEVELDBENTITYDATASTREAMPROVICER_H
#define LEVELDBENTITYDATASTREAMPROVICER_H

#include "abstractentitydatastreamprovider.h"
#include "leveldb/db.h"

namespace upns
{

class LevelDBEntityDataStreamProvider : public AbstractEntityDataStreamProvider
{
public:
    LevelDBEntityDataStreamProvider(leveldb::DB *db, const std::string &key);
    bool isCached();
    bool isReadWriteSame();
    upnsIStream *startRead(upnsuint64 start, upnsuint64 len);
    void endRead(upnsIStream *strm);
    upnsOStream *startWrite(upnsuint64 start, upnsuint64 len);
    void endWrite(upnsOStream *strm);
private:
    leveldb::DB *m_db;
    std::string m_key;
};

}

#endif
