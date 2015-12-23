#ifndef _FILELAYERDATASTREAMPROVIDER_H__
#define _FILELAYERDATASTREAMPROVIDER_H__

#include "abstractentitydatastreamprovider.h"
#include "leveldb/db.h"

namespace upns
{

class FileEntityDataStreamProvider : public AbstractEntityDataStreamProvider
{
public:
    FileEntityDataStreamProvider(leveldb::DB *db, const std::string &key);
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
