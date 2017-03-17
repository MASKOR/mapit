#include "zmqentitydatastreamprovider.h"
#include <sstream>
#include <fstream>
#include <cstdio> // for tempfile

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#define FILEMODE S_IRWXU | S_IRGRP | S_IROTH

struct ZmqReadWriteHandle
{
    upns::upnsuint64 offset;
    upns::upnsuint64 length;
};

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


void *mapFile(const uint64_t &len, const uint64_t &start, const char *filename, int &handle, int flag, int privShared, int openflag)
{
    if ((handle = open(filename, openflag | O_CREAT, FILEMODE)) < 0)
    {
        log_error("Error in file opening");
        return nullptr;
    }
//    result = lseek(handle, FILESIZE-1, SEEK_SET);
//    if (result == -1) {
//        close(handle);
//        log_error("Error calling lseek() to 'stretch' the file");
//        return nullptr;
//    }
    void *addr;
    if ((addr = mmap(NULL, len, PROT_READ, MAP_SHARED, handle, start)) == MAP_FAILED)
    {
        log_error("Error in file mapping");
        return nullptr;
    }
    return addr;
}
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
    upns::upnsuint64 outLength;
    return new MemoryReaderDeleter(startRead(start, length, outLength), outLength);
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
    endWrite(buf.data(), buf.length(), ostrm->getOffset());
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

const void *upns::ZmqEntitydataStreamProvider::startReadPointer(ReadWriteHandle &handle, upns::upnsuint64 start, upns::upnsuint64 len)
{
    upnsuint64 outLen;
    char* ptr = startRead(start, len, outLen);
    return static_cast<void *>(ptr);
}

void upns::ZmqEntitydataStreamProvider::endReadPointer(const void *ptr, ReadWriteHandle &handle)
{
    delete [] static_cast<const char*>(ptr);
}

void *upns::ZmqEntitydataStreamProvider::startWritePointer(ReadWriteHandle &handle, upns::upnsuint64 start, upns::upnsuint64 len)
{
    ZmqReadWriteHandle *zHandle(new ZmqReadWriteHandle);
    zHandle->offset = start;
    handle = static_cast<void *>(zHandle);
    return new char[len];
}

void upns::ZmqEntitydataStreamProvider::endWritePointer(void *ptr, ReadWriteHandle &handle)
{
    ZmqReadWriteHandle *zHandle(static_cast<ZmqReadWriteHandle*>(handle));
    endWrite(static_cast<const char*>(ptr), zHandle->length, zHandle->offset);
    delete [] static_cast<char *>(ptr);
    delete zHandle;
}

char *upns::ZmqEntitydataStreamProvider::startRead(upns::upnsuint64 start, upns::upnsuint64 length, upns::upnsuint64 &outLength)
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
        recvlen = 1; // This is used to make allocation of memory possible. Use m_entityLength for the exact size.
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

    // receive data frames
    // also receive empty frames to not disturb following receives.
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
    outLength = recvlen;
    return buf;
}

void upns::ZmqEntitydataStreamProvider::endWrite(const char *memory, const upnsuint64 &length, const upnsuint64 &offset)
{
    if(length > m_entityLength)
    {
        m_entityLength = length;
    }
    std::unique_ptr<upns::RequestStoreEntity> req(new upns::RequestStoreEntity);
    req->set_checkout(m_checkoutName);
    req->set_path(m_pathOrOid);
    req->set_offset(offset);
    req->set_sendlength(length);
    req->set_entitylength(m_entityLength);
    // Note: Entitytype can not be set here!
    m_node->send(std::move(req), ZMQ_SNDMORE);
    m_node->send_raw_body( reinterpret_cast<const unsigned char*>(memory), length ); //TODO: add zero copy support!

    upnsSharedPointer<upns::ReplyStoreEntity> rep(m_node->receive<upns::ReplyStoreEntity>());
    if(rep->status() != upns::ReplyStoreEntity::SUCCESS)
    {
        log_error("received error from server when asked for entitydata");
    }
}

upns::upnsString upns::ZmqEntitydataStreamProvider::startReadFile(upns::ReadWriteHandle &handle)
{
    // tmpnam is cross platform but has the chance of returning an invalid temp filename:
    // if another process also requested a tmpfile but has not yet created it.

    char *filename = new char[L_tmpnam];
    std::string tmpfilename = std::tmpnam(filename);
    handle = static_cast<ReadWriteHandle>(filename);
    std::ofstream outfile (tmpfilename,std::ofstream::binary);

    upnsuint64 outLen;
    char* ptr = startRead(0, 0, outLen);
    outfile.write(ptr, outLen);
    delete [] ptr;
    return tmpfilename;
}

void upns::ZmqEntitydataStreamProvider::endReadFile(upns::ReadWriteHandle &handle)
{
    char *filename = static_cast<char*>(handle);
    std::remove(filename);
}

upns::upnsString upns::ZmqEntitydataStreamProvider::startWriteFile(upns::ReadWriteHandle &handle)
{
    // tmpnam is cross platform but has the chance of returning a temp filename,
    // if another process also requested a tmpfile but has not yet created it.

    char *filename = new char[L_tmpnam];
    std::string tmpfilename = std::tmpnam(filename);
    handle = static_cast<ReadWriteHandle>(filename);
    return tmpfilename;
}

void upns::ZmqEntitydataStreamProvider::endWriteFile(upns::ReadWriteHandle &handle)
{
    char *filename = static_cast<char*>(handle);
    int filedescriptor;

    std::ifstream is(filename);
    is.seekg(0, std::ios::beg);
    is.seekg(0, std::ios::end);
    std::stringstream::pos_type size = is.tellg();
    void *addr_void = mapFile(size, 0, filename, filedescriptor, PROT_READ, MAP_SHARED, O_RDONLY);
    char *addr = static_cast<char *>(addr_void);

    endWrite(addr, size, 0);

    if (munmap(addr, size) == -1)
    {
        log_error("Error un-mmapping the file");
        close(filedescriptor);
    }
    std::remove(filename);
}

upns::AbstractEntitydataProvider::ReadWriteType upns::ZmqEntitydataStreamProvider::preferredReadType()
{
    return ReadWritePointer;
}

upns::AbstractEntitydataProvider::ReadWriteType upns::ZmqEntitydataStreamProvider::preferredWriteType()
{
    return ReadWritePointer;
}
