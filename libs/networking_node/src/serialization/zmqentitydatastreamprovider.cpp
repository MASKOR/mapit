#include "zmqentitydatastreamprovider.h"

class MemoryReaderDeleter : public std::istringstream
{
public:
    MemoryReaderDeleter(char* data, size_t size)
        :m_data(data),
         std::istringstream(std::string(m_data, size))
    {}
    virtual ~MemoryReaderDeleter()
    {
        delete m_data;
    }
private:
    char* m_data;
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


upns::ZmqEntitydataStreamProvider::ZmqEntitydataStreamProvider(upns::upnsString checkoutName, upns::upnsString pathOrOid, ZmqNode *node)
    :m_checkoutName(checkoutName),
     m_pathOrOid(pathOrOid),
     m_node(node),
     m_e(nullptr),
     m_ed(nullptr)
{

}

bool upns::ZmqEntitydataStreamProvider::isCached()
{
    return false; // such uncached. much network. latency intensified.
}

bool upns::ZmqEntitydataStreamProvider::isReadWriteSame()
{

}

upns::upnsIStream *upns::ZmqEntitydataStreamProvider::startRead(upns::upnsuint64 start, upns::upnsuint64 len)
{
    //TODO: mixed unit / int.
    int64_t size = len == 0 ? -1 : static_cast<int64_t>(len);
    if(size == 0)
    {
        size = -1;
    }
    std::unique_ptr<upns::RequestEntitydata> req(new upns::RequestEntitydata);
    req->set_checkout(m_checkoutName);
    req->set_entitypath(m_pathOrOid);
    req->set_offset(start);
    req->set_maxlength(size);
    m_node->send(std::move(req));
    char *buf = new char[size];
    m_ed = upns::upnsSharedPointer<upns::ReplyEntitydata>(m_node->receive<upns::ReplyEntitydata>());
    // Receive empty frames to not disturb following receives.
    if(m_node->has_more())
    {
        size_t offset = 0;
        do {
            offset += m_node->receive_raw_body(buf+offset, sizeof(buf)-offset);
        } while (m_node->has_more());
        if(offset != size)
        {
            log_error("Entitydata does not have expected size. It may be corrupt. " + m_pathOrOid);
        }
    }
    return new MemoryReaderDeleter(buf, sizeof(buf));
}

void upns::ZmqEntitydataStreamProvider::endRead(upns::upnsIStream *strm)
{
    delete strm;
}

upns::upnsOStream *upns::ZmqEntitydataStreamProvider::startWrite(upns::upnsuint64 start, upns::upnsuint64 len)
{

}

void upns::ZmqEntitydataStreamProvider::endWrite(upns::upnsOStream *strm)
{

}

upns::upnsuint64 upns::ZmqEntitydataStreamProvider::getStreamSize() const
{

}

void upns::ZmqEntitydataStreamProvider::setStreamSize(upns::upnsuint64)
{

}

upns::LockHandle upns::ZmqEntitydataStreamProvider::lock()
{

}

void upns::ZmqEntitydataStreamProvider::unlock(upns::LockHandle)
{

}
