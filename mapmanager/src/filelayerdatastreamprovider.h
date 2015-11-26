#ifndef _FILELAYERDATASTREAMPROVIDER_H__
#define _FILELAYERDATASTREAMPROVIDER_H__

#include "abstractlayerdatastreamprovider.h"
#include "leveldb/db.h"

namespace upns
{

class FileLayerDataStreamProvider : public AbstractLayerDataStreamProvider
{
public:
    FileLayerDataStreamProvider(leveldb::DB *db, const std::string &key);
    bool isCached();
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