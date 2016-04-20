#include "leveldbentitydatastreamprovider.h"
#include <sstream>
#include <string>

namespace upns {

LevelDBEntityDataStreamProvider::LevelDBEntityDataStreamProvider(leveldb::DB *db, const std::string &readkey, const std::string &writekey)
    :m_db(db),
     m_readkey(readkey),
     m_writekey(writekey)
{
    assert(m_db != NULL);
    //assert(!m_readkey.empty());
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
    it->Seek(m_readkey);
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
    //TODO: copying if not the whole stream was read
    std::ostringstream *osstrm = static_cast<std::ostringstream *>(strm);
    m_db->Put(leveldb::WriteOptions(), m_writekey, osstrm->str());
    delete strm;
}

upnsuint64 LevelDBEntityDataStreamProvider::getStreamSize() const
{
    leveldb::Iterator* it = m_db->NewIterator(leveldb::ReadOptions());
    it->Seek(m_writekey);
    assert(it->Valid());
    leveldb::Slice slice = it->value();
    delete it;
    return slice.size();
}

void LevelDBEntityDataStreamProvider::setStreamSize(upnsuint64 streamSize)
{
    leveldb::Iterator* it = m_db->NewIterator(leveldb::ReadOptions());
    it->Seek(m_writekey);
    assert(it->Valid());
    leveldb::Slice slice = it->value();
    std::string data(slice.data());
    data.resize(streamSize); //TODO: This may not be the most efficient way.
    m_db->Put(leveldb::WriteOptions(), m_writekey, data);
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
