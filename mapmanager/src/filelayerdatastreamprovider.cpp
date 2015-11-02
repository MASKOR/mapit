#include "filelayerdatastreamprovider.h"
#include <sstream>
#include <string>

namespace upns {

FileLayerDataStreamProvider::FileLayerDataStreamProvider(leveldb::DB *db, const leveldb::Slice &key)
    :m_db(db),
     m_key(key)
{
    assert(m_db != NULL);
    assert(!m_key.empty());
}

bool FileLayerDataStreamProvider::isCached()
{
    return true;
}

upnsIStream* upns::FileLayerDataStreamProvider::startRead(upnsuint64 start, upnsuint64 len)
{
    leveldb::Iterator* it = m_db->NewIterator(leveldb::ReadOptions());
    it->Seek(m_key);
    assert(it->Valid());
    leveldb::Slice slice = it->value();
    assert(start < slice.size());
    assert(start + len < slice.size());

    //TODO: add locking
    std::string str(slice.data() + start);
    std::istringstream *isstr = new std::istringstream( str );
    return isstr;
}

void FileLayerDataStreamProvider::endRead(upnsIStream *strm)
{
    //TODO: add locking
    delete strm;
}

upnsOStream *upns::FileLayerDataStreamProvider::startWrite(upnsuint64 start, upnsuint64 len)
{
    //TODO: add locking
    return new std::ostringstream(std::string());
}

void FileLayerDataStreamProvider::endWrite(upnsOStream *strm)
{
    //TODO: add locking
    std::ostringstream *osstrm = static_cast<std::ostringstream *>(strm);
    m_db->Put(leveldb::WriteOptions(), m_key, osstrm->str());
    delete strm;
}

}
