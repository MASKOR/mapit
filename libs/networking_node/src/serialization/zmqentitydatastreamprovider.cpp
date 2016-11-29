#include "zmqentitydatastreamprovider.h"
#include <sstream>

class MemoryReaderDeleter : public std::istringstream
{
public:
    MemoryReaderDeleter(char* data, size_t size)
        :m_data(data),
         m_mem(m_data, size)
    {
        str(m_mem);
        clear();
    }
    virtual ~MemoryReaderDeleter()
    {
        delete [] m_data;
    }
private:
    char* m_data;
    std::string m_mem;
};

class MyWriter : public std::ostringstream
{
    size_t m_offset;
public:
    MyWriter(size_t offset):m_offset(offset)
    {}
    size_t getOffset(){return m_offset;}
    size_t getSize()
    {
        seekp(0, std::ios::beg);
        seekp(0, std::ios::end);
        std::stringstream::pos_type size = tellp();
        return size;
    }
};


upns::ZmqEntitydataStreamProvider::ZmqEntitydataStreamProvider(upns::upnsString checkoutName, upns::upnsString pathOrOid, ZmqProtobufNode *node)
    :m_checkoutName(checkoutName),
     m_pathOrOid(pathOrOid),
     m_node(node),
     m_entityLength(0)
{

}

bool upns::ZmqEntitydataStreamProvider::isCached()
{
    return false; // such uncached. much network. latency intensified.
}

bool upns::ZmqEntitydataStreamProvider::isReadWriteSame()
{
    // different memory adresses to send and receive
    return false;
}

#include <iomanip>

upns::upnsIStream *upns::ZmqEntitydataStreamProvider::startRead(upns::upnsuint64 start, upns::upnsuint64 length)
{

    const uint64_t defaultBufferSize =  500ul*1024ul*1024ul; // 500 MB
    const uint64_t maxBufferSize = 2ul*1024ul*1024ul*1024ul; // 2 GB
    if(length == 0)
    {
        length = maxBufferSize;
    }
    std::unique_ptr<upns::RequestEntitydata> req(new upns::RequestEntitydata);
    req->set_checkout(m_checkoutName);
    req->set_entitypath(m_pathOrOid);
    req->set_offset(start);
    req->set_maxlength(maxBufferSize);
    m_node->send(std::move(req));
    upnsSharedPointer<upns::ReplyEntitydata> rep(m_node->receive<upns::ReplyEntitydata>());
    if(rep->status() != upns::ReplyEntitydata::SUCCESS)
    {
        log_error("received error from server when asked for entitydata");
        return nullptr;
    }
    m_entityLength = rep->entitylength();
    uint64_t recvlen = rep->receivedlength();
    if(recvlen == 0)
    {
        log_info("Received empty entitydata (length: 0, entity: " + m_pathOrOid + ")");
        recvlen = 1;
    }
    if(recvlen < 0)
    {
        log_warn("Entitydata with negative size received. (length: " + std::to_string(recvlen) + ")");
        recvlen = defaultBufferSize;
    }
    if(recvlen > maxBufferSize)
    {
        log_warn("Entitydata length for single req/rep roundtrip exceeded maximum size. (length: " + std::to_string(recvlen) + ")");
        recvlen = maxBufferSize;
    }
    char *buf = new char[recvlen]; // deleted by MemoryReaderDeleter

    // receive empty frames to not disturb following receives.
    if(m_node->has_more())
    {
        size_t offset = 0;
        do
        {
            offset += m_node->receive_raw_body(buf+offset, recvlen-offset);
        } while (m_node->has_more() && offset <= recvlen);

        if( offset != recvlen && !(offset == 0 && recvlen == 1) )
        {
            log_error("Entitydata does not have expected size. It may be corrupt. (entity: " + m_pathOrOid + ")");
        }
        if( m_node->has_more() )
        {
            log_error("Received unexpected network part. Entitydata may be corrupt. (entity: " + m_pathOrOid + ")");
        }
    }
    return new MemoryReaderDeleter(buf, recvlen);
}

void upns::ZmqEntitydataStreamProvider::endRead(upns::upnsIStream *strm)
{
    delete strm;
}

upns::upnsOStream *upns::ZmqEntitydataStreamProvider::startWrite(upns::upnsuint64 start, upns::upnsuint64 len)
{
    // Note: len is absolutely not used here
    return new MyWriter(start);
}

void upns::ZmqEntitydataStreamProvider::endWrite(upns::upnsOStream *strm)
{
    MyWriter *ostrm(static_cast<MyWriter*>(strm));
    std::string buf(ostrm->str());
    if(buf.length() > m_entityLength)
    {
        m_entityLength = buf.length();
    }
    std::unique_ptr<upns::RequestStoreEntity> req(new upns::RequestStoreEntity);
    req->set_checkout(m_checkoutName);
    req->set_path(m_pathOrOid);
    req->set_offset(ostrm->getOffset());
    req->set_sendlength(buf.length());
    req->set_entitylength(m_entityLength);
    // Note: Entitytype can not be set here!
    m_node->send(std::move(req), ZMQ_SNDMORE);
    m_node->send_raw_body( reinterpret_cast<const unsigned char*>(buf.data()), buf.length() ); //TODO: add zero copy support!
    delete ostrm;


    upnsSharedPointer<upns::ReplyStoreEntity> rep(m_node->receive<upns::ReplyStoreEntity>());
    if(rep->status() != upns::ReplyStoreEntity::SUCCESS)
    {
        log_error("received error from server when asked for entitydata");
    }
}

upns::upnsuint64 upns::ZmqEntitydataStreamProvider::getStreamSize() const
{
    if(m_entityLength == 0)
    {
        std::unique_ptr<upns::RequestEntitydata> req(new upns::RequestEntitydata);
        req->set_checkout(m_checkoutName);
        req->set_entitypath(m_pathOrOid);
        req->set_offset(0ul);
        req->set_maxlength(0ul);
        m_node->send(std::move(req));
        upnsSharedPointer<upns::ReplyEntitydata> rep(m_node->receive<upns::ReplyEntitydata>());
        if(rep->status() != upns::ReplyEntitydata::SUCCESS)
        {
            log_error("received error from server when storing entitydata");
        }
        else
        {
            m_entityLength = rep->entitylength();
        }
    }
    return m_entityLength;
}

void upns::ZmqEntitydataStreamProvider::setStreamSize(upns::upnsuint64 entitylength)
{
    m_entityLength = entitylength;
    std::unique_ptr<upns::RequestStoreEntity> req(new upns::RequestStoreEntity);
    req->set_checkout(m_checkoutName);
    req->set_path(m_pathOrOid);
    req->set_offset(0ul);
    req->set_sendlength(0ul);
    req->set_entitylength(m_entityLength);
    m_node->send(std::move(req));
    upnsSharedPointer<upns::ReplyStoreEntity> rep(m_node->receive<upns::ReplyStoreEntity>());
    if(rep->status() != upns::ReplyStoreEntity::SUCCESS)
    {
        log_error("received error from server when setting entitydata length");
    }
}

upns::LockHandle upns::ZmqEntitydataStreamProvider::lock()
{

}

void upns::ZmqEntitydataStreamProvider::unlock(upns::LockHandle)
{

}
