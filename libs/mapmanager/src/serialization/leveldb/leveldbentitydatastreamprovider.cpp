#include "leveldbentitydatastreamprovider.h"
#include <sstream>
#include <string>
//#include <boost/iostreams/stream.hpp>

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

//class SliceReader : public boost::archive::binary_iarchive
//{
//    typedef boost::iostreams::basic_array_source<char> Device;
//public:
//    SliceReader(leveldb::Iterator* iter, size_t start, size_t size)
//        :m_iter(iter),
//         m_buffer(iter->value().data() + start, size),
//         boost::archive::binary_iarchive(m_buffer, boost::archive::no_header)
//    {}
//    virtual ~SliceReader()
//    {
//        delete m_iter;
//    }

//private:
//    boost::iostreams::stream_buffer<Device> m_buffer;
//    leveldb::Iterator* m_iter;
//};

//class MyWriter : public boost::archive::binary_oarchive
//{
//    typedef boost::iostreams::basic_array_source<char> Device;
//public:
//    MyWriter()
//         :m_ostr(),
//           boost::archive::binary_oarchive(m_ostr, boost::archive::no_header)
//    {}

//    std::stringstream m_ostr;
//};

class SliceReader : public std::istringstream
{
public:
    SliceReader(leveldb::Iterator* iter, size_t start, size_t size)
        :m_iter(iter),
         std::istringstream(std::string(iter->value().data() + start, size)) //TODO: This copies data!?
    {}
    virtual ~SliceReader()
    {
        delete m_iter;
    }

private:
    leveldb::Iterator* m_iter;
};

class MyWriter : public std::ostringstream
{
public:
    MyWriter()
    {}
    size_t getSize()
    {
        seekp(0, std::ios::beg);
        seekp(0, std::ios::end);
        std::stringstream::pos_type size = tellp();
        return size;
    }
};

upnsIStream* upns::LevelDBEntityDataStreamProvider::startRead(upnsuint64 start, upnsuint64 len)
{
    leveldb::Iterator* it = m_db->NewIterator(leveldb::ReadOptions());
    it->Seek(m_readkey);
    assert(it->Valid());
    leveldb::Slice slice = it->value();
    //delete it;
    assert(start <= slice.size());
    if( len == 0 ) len = slice.size();
    assert(start + len <= slice.size());

    //TODO: add locking
    //std::string str(slice.data() + start, len);
    //std::cout << "DBG: read wrk" << m_writekey << " from rdk" << m_readkey << ", size: " << len << std::endl;
//    std::basic_istream<const char> *isstr = new std::basic_istream<const char>(slice.data(), len);

    //isstr->rdbuf()->pubsetbuf(str, static_cast<size_t>(len));

    SliceReader *archive = new SliceReader(it, start, len);
    return archive;
}

void LevelDBEntityDataStreamProvider::endRead(upnsIStream *strm)
{
    //TODO: add locking
    delete strm;
}

upnsOStream *upns::LevelDBEntityDataStreamProvider::startWrite(upnsuint64 start, upnsuint64 len)
{
    //TODO: add locking
    //return new std::ostringstream(std::string());
//    typedef boost::iostreams::basic_array_source<char> Device;
//    boost::iostreams::stream_buffer<Device> buffer();
    return new MyWriter();
}

void LevelDBEntityDataStreamProvider::endWrite(upnsOStream *strm)
{
    //TODO: add locking
    //TODO: copying if not the whole stream was read
//    std::ostringstream *osstrm = static_cast<std::ostringstream *>(strm);
//    std::cout << "size written :" << size << ", dat: " << osstrm->str() <<  std::endl;
//    //osstrm->seekg(0, std::ios::beg);
    MyWriter *ostrm(static_cast<MyWriter*>(strm));
//    char* fatbuf = new char[ostrm->getSize()];
//    ostrm->str().copy(fatbuf, ostrm->getSize());
//    ostrm.seekp(0, std::ios::end);
//    std::stringstream::pos_type size = ostrm.tellp();
    //std::string test();
    std::string buf(ostrm->str());
    leveldb::Slice val(buf.data(), ostrm->getSize());
//    //std::cout << "DBG: write " << m_writekey << " from " << m_readkey << ", size: " << str.length() << std::endl;
    m_db->Put(leveldb::WriteOptions(), m_writekey, val);
    delete strm;
//    delete fatbuf;
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
