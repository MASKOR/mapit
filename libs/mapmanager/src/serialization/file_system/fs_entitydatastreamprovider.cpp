#include "fs_entitydatastreamprovider.h"
#include <fstream>
#include <string>
#include <sstream>
#include <assert.h>

namespace upns {

FileSystemEntitydataStreamProvider::FileSystemEntitydataStreamProvider(const std::string &filenameRead, const std::string &filenameWrite)
    :m_filenameRead(filenameRead),
     m_filenameWrite(filenameWrite)
{
    assert(!m_filenameRead.empty());
}

bool FileSystemEntitydataStreamProvider::isCached()
{
    return true;
}

bool FileSystemEntitydataStreamProvider::isReadWriteSame()
{
    return true;
}

upnsIStream* upns::FileSystemEntitydataStreamProvider::startRead(upnsuint64 start, upnsuint64 len)
{
    std::ifstream *is = new std::ifstream(m_filenameRead);
    //TODO: SEEK

    return is;
}

void FileSystemEntitydataStreamProvider::endRead(upnsIStream *strm)
{
    delete strm;
}

upnsOStream *upns::FileSystemEntitydataStreamProvider::startWrite(upnsuint64 start, upnsuint64 len)
{
    //TODO: seek, overwrite
    return new std::ofstream(m_filenameWrite);
}

void FileSystemEntitydataStreamProvider::endWrite(upnsOStream *strm)
{
    assert(static_cast<std::ofstream*>(strm)->is_open());
    strm->flush();
    static_cast<std::ofstream*>(strm)->close();
    delete strm;
}

upnsuint64 FileSystemEntitydataStreamProvider::getStreamSize() const
{
    std::ifstream is(m_filenameRead);
    is.seekg(0, std::ios::beg);
    is.seekg(0, std::ios::end);
    std::stringstream::pos_type size = is.tellg();
    return size;
}

void FileSystemEntitydataStreamProvider::setStreamSize(upnsuint64 streamSize)
{
    std::ofstream os(m_filenameWrite);
    // TODO
}

LockHandle FileSystemEntitydataStreamProvider::lock()
{
    //TODO: impl
    return 0;
}

void FileSystemEntitydataStreamProvider::unlock(LockHandle)
{
    //TODO: impl
}

}
