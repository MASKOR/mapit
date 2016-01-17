#include "leveldbentitydatastreamprovider.h"
#include <sstream>
#include <string>

namespace upns {

LevelDBEntityDataStreamProvider::LevelDBEntityDataStreamProvider(leveldb::DB *db, const std::string &key)
    :m_db(db),
     m_key(key)
{
    assert(m_db != NULL);
    assert(!m_key.empty());
}

bool LevelDBEntityDataStreamProvider::isCached()
{
    return true;
}

bool LevelDBEntityDataStreamProvider::isReadWriteSame()
{
    return true;
}

upnsIStream* upns::LevelDBEntityDataStreamProvider::startRead(upnsuint64 start, upnsuint64 len)
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

void LevelDBEntityDataStreamProvider::endRead(upnsIStream *strm)
{
    //TODO: add locking
    delete strm;
}

upnsOStream *upns::LevelDBEntityDataStreamProvider::startWrite(upnsuint64 start, upnsuint64 len)
{
    //TODO: add locking
    return new std::ostringstream(std::string());
}

void LevelDBEntityDataStreamProvider::endWrite(upnsOStream *strm)
{
    //TODO: add locking
    std::ostringstream *osstrm = static_cast<std::ostringstream *>(strm);
    m_db->Put(leveldb::WriteOptions(), m_key, osstrm->str());
    delete strm;
}

upnsuint64 LevelDBEntityDataStreamProvider::getStreamSize() const
{
    leveldb::Iterator* it = m_db->NewIterator(leveldb::ReadOptions());
    it->Seek(m_key);
    assert(it->Valid());
    leveldb::Slice slice = it->value();
    delete it;
    return slice.size();
}

void LevelDBEntityDataStreamProvider::setStreamSize(upnsuint64 streamSize)
{
    leveldb::Iterator* it = m_db->NewIterator(leveldb::ReadOptions());
    it->Seek(m_key);
    assert(it->Valid());
    leveldb::Slice slice = it->value();
    std::string data(slice.data());
    data.resize(streamSize); //TODO: This may not be the most efficient way.
    m_db->Put(leveldb::WriteOptions(), m_key, data);
    delete it;
}

LockHandle LevelDBEntityDataStreamProvider::lock()
{
    //TODO: impl
    return 0;
}

void LevelDBEntityDataStreamProvider::unlock(LockHandle)
{
    //TODO: impl
}

}
