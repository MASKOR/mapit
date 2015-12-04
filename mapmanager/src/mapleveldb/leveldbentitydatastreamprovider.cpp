#include "mapleveldb/leveldbentitydatastreamprovider.h"
#include <sstream>
#include <string>

namespace upns {

FileEntityDataStreamProvider::FileEntityDataStreamProvider(leveldb::DB *db, const std::string &key)
    :m_db(db),
     m_key(key)
{
    assert(m_db != NULL);
    assert(!m_key.empty());
}

bool FileEntityDataStreamProvider::isCached()
{
    return true;
}

upnsIStream* upns::FileEntityDataStreamProvider::startRead(upnsuint64 start, upnsuint64 len)
{
    leveldb::Iterator* it = m_db->NewIterator(leveldb::ReadOptions());
    it->Seek(m_key);
    assert(it->Valid());
    leveldb::Slice slice = it->value();
    delete it;
    assert(start < slice.size());
    assert(start + len < slice.size());

    //TODO: add locking
    if( len == 0 ) len = slice.size();
    std::string str(slice.data() + start, len);
    std::istringstream *isstr = new std::istringstream( str );
    return isstr;
}

void FileEntityDataStreamProvider::endRead(upnsIStream *strm)
{
    //TODO: add locking
    delete strm;
}

upnsOStream *upns::FileEntityDataStreamProvider::startWrite(upnsuint64 start, upnsuint64 len)
{
    //TODO: add locking
    return new std::ostringstream(std::string());
}

void FileEntityDataStreamProvider::endWrite(upnsOStream *strm)
{
    //TODO: add locking
    std::ostringstream *osstrm = static_cast<std::ostringstream *>(strm);
    m_db->Put(leveldb::WriteOptions(), m_key, osstrm->str());
    delete strm;
}

}
