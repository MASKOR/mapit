//#include "mapleveldb/leveldbentitydatastreamprovider.h"
//#include <sstream>
//#include <string>

//namespace upns {

//FileEntitydataStreamProvider::FileEntitydataStreamProvider(leveldb::DB *db, const std::string &key)
//    :m_db(db),
//     m_key(key)
//{
//    assert(m_db != NULL);
//    assert(!m_key.empty());
//}

//bool FileEntitydataStreamProvider::isCached()
//{
//    return true;
//}

//bool FileEntitydataStreamProvider::isReadWriteSame()
//{
//    return true;
//}

//upnsIStream* upns::FileEntitydataStreamProvider::startRead(upnsuint64 start, upnsuint64 len)
//{
//    leveldb::Iterator* it = m_db->NewIterator(leveldb::ReadOptions());
//    it->Seek(m_key);
//    assert(it->Valid());
//    leveldb::Slice slice = it->value();
//    delete it;
//    assert(start < slice.size());
//    assert(start + len < slice.size());

//    //TODO: add locking
//    if( len == 0 ) len = slice.size();
//    std::string str(slice.data() + start, len);
//    std::istringstream *isstr = new std::istringstream( str );
//    return isstr;
//}

//void FileEntitydataStreamProvider::endRead(upnsIStream *strm)
//{
//    //TODO: add locking
//    delete strm;
//}

//upnsOStream *upns::FileEntitydataStreamProvider::startWrite(upnsuint64 start, upnsuint64 len)
//{
//    //TODO: add locking
//    return new std::ostringstream(std::string());
//}

//void FileEntitydataStreamProvider::endWrite(upnsOStream *strm)
//{
//    //TODO: add locking
//    std::ostringstream *osstrm = static_cast<std::ostringstream *>(strm);
//    m_db->Put(leveldb::WriteOptions(), m_key, osstrm->str());
//    delete strm;
//}

//}
