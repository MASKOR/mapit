/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2017 Tobias Neumann	<t.neumann@fh-aachen.de>
 *
******************************************************************************/

/*  This file is part of mapit.
 *
 *  Mapit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Mapit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with mapit.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "zmqentitydatastreamprovider.h"
#include <sstream>
#include <fstream>
#include <cstdio> // for tempfile
#include <boost/iostreams/device/mapped_file.hpp>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h> // tempnam
#include <features.h>
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

upns::ZmqEntitydataStreamProvider::ZmqEntitydataStreamProvider(std::string checkoutName, std::string pathOrOid, ZmqProtobufNode *node)
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

void upns::ZmqEntitydataStreamProvider::endRead(upns::upnsIStream *&strm)
{
    delete strm;
}

upns::upnsOStream *upns::ZmqEntitydataStreamProvider::startWrite(upns::upnsuint64 start, upns::upnsuint64 len)
{
    // Note: len is absolutely not used here
    return new MyWriter(start);
}

void upns::ZmqEntitydataStreamProvider::endWrite(upns::upnsOStream *&strm)
{
    MyWriter *ostrm(static_cast<MyWriter*>(strm));
    std::string buf(ostrm->str());
    endWrite(buf.data(), buf.length(), ostrm->getOffset());
}

upns::upnsuint64 upns::ZmqEntitydataStreamProvider::getStreamSize() const
{
    if(m_entityLength == 0)
    {
        std::unique_ptr<RequestEntitydata> req(new RequestEntitydata);
        req->set_checkout(m_checkoutName);
        req->set_entitypath(m_pathOrOid);
        req->set_offset(0ul);
        req->set_maxlength(0ul);
        try
        {
            m_node->prepareForwardComChannel();
            m_node->send(std::move(req));
            m_node->prepareBackComChannel();
            std::shared_ptr<ReplyEntitydata> rep(m_node->receive<ReplyEntitydata>());
            if(rep->status() != ReplyEntitydata::SUCCESS)
            {
                log_error("received error from server when storing entitydata");
            }
            else
            {
                m_entityLength = rep->entitylength();
            }
        }
        catch( zmq::error_t err)
        {
            log_error("network error for setStreamSize: " + err.what());
            return 0;
        }
    }
    return m_entityLength;
}

void upns::ZmqEntitydataStreamProvider::setStreamSize(upns::upnsuint64 entitylength)
{
    m_entityLength = entitylength;
    std::unique_ptr<RequestStoreEntity> req(new RequestStoreEntity);
    req->set_checkout(m_checkoutName);
    req->set_path(m_pathOrOid);
    req->set_offset(0ul);
    req->set_sendlength(0ul);
    req->set_entitylength(m_entityLength);
    try
    {
        m_node->prepareForwardComChannel();
        m_node->send(std::move(req));
        m_node->prepareBackComChannel();
        std::shared_ptr<ReplyStoreEntity> rep(m_node->receive<ReplyStoreEntity>());
        if(rep->status() != ReplyStoreEntity::SUCCESS)
        {
            log_error("received error from server when setting entitydata length");
        }
    }
    catch( zmq::error_t err)
    {
        log_error("network error for setStreamSize: " + err.what());
    }
}

upns::LockHandle upns::ZmqEntitydataStreamProvider::lock()
{
    return 0;
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
    std::unique_ptr<RequestEntitydata> req(new RequestEntitydata);
    req->set_checkout(m_checkoutName);
    req->set_entitypath(m_pathOrOid);
    req->set_offset(start);
    req->set_maxlength(maxBufferSize);
    std::shared_ptr<ReplyEntitydata> rep;
    try {
        m_node->prepareForwardComChannel();
        m_node->send(std::move(req));
        m_node->prepareBackComChannel();
        rep.reset(m_node->receive<ReplyEntitydata>());
    }
    catch( zmq::error_t err)
    {
        log_error("network error when asked for entitydata: " + err.what());
        return nullptr;
    }
    if(rep && rep->status() != ReplyEntitydata::SUCCESS)
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
            try {
                offset += m_node->receive_raw_body(buf+offset, recvlen-offset);
            }
            catch( zmq::error_t err)
            {
                log_error("network error when asked for entitydata: " + err.what());
                return nullptr;
            }
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
    std::unique_ptr<RequestStoreEntity> req(new RequestStoreEntity);
    req->set_checkout(m_checkoutName);
    req->set_path(m_pathOrOid);
    req->set_offset(offset);
    req->set_sendlength(length);
    req->set_entitylength(m_entityLength);

    try {
        m_node->prepareForwardComChannel();
        m_node->send(std::move(req), ZMQ_SNDMORE);
        m_node->send_raw_body( reinterpret_cast<const unsigned char*>(memory), length ); //TODO: add zero copy support!
        // Note: Entitytype can not be set here!

        m_node->prepareBackComChannel();
        std::shared_ptr<ReplyStoreEntity> rep(m_node->receive<ReplyStoreEntity>());
        if(rep->status() != ReplyStoreEntity::SUCCESS)
        {
            log_error("received error from server when asked for entitydata");
        }
    }
    catch( zmq::error_t err)
    {
        log_error("network error while endWrite: " + err.what());
    }
}

std::string upns::ZmqEntitydataStreamProvider::startReadFile(upns::ReadWriteHandle &handle)
{
    // tmpnam is cross platform but has the chance of returning an invalid temp filename:
    // if another process also requested a tmpfile but has not yet created it.

    char *filename = new char[L_tmpnam];

    // TODO: use something save. This is the only crossplatform solution. "tempnam" (with e) is in the making!
    // mkstemp is no option, as we can not obtain the filename. Filename is needed for layertypes (e.g. PCL, which uses filenames).
    std::string tmpfilename = std::tmpnam(filename);

    handle = static_cast<ReadWriteHandle>(filename);
    std::ofstream outfile (tmpfilename, std::ofstream::binary);

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

std::string upns::ZmqEntitydataStreamProvider::startWriteFile(upns::ReadWriteHandle &handle)
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

    boost::iostreams::mapped_file outfile;
    outfile.open(filename, boost::iostreams::mapped_file::priv, size);
    void *addr_void = outfile.data();
    char *addr = static_cast<char *>(addr_void);

    endWrite(addr, size, 0);

    outfile.close();
//    if (munmap(addr, size) == -1)
//    {
//        log_error("Error un-mmapping the file");
//        close(filedescriptor);
//    }
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
