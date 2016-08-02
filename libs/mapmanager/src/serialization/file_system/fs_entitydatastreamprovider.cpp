#include "fs_entitydatastreamprovider.h"
#include <fstream>
#include <string>

namespace upns {

FileSystemEntityDataStreamProvider::FileSystemEntityDataStreamProvider(const std::string &filenameRead, const std::string &filenameWrite)
    :m_filenameRead(filenameRead),
     m_filenameWrite(filenameWrite)
{
    assert(!m_filenameRead.empty());
}

bool FileSystemEntityDataStreamProvider::isCached()
{
    return true;
}

bool FileSystemEntityDataStreamProvider::isReadWriteSame()
{
    return true;
}

upnsIStream* upns::FileSystemEntityDataStreamProvider::startRead(upnsuint64 start, upnsuint64 len)
{
    std::ifstream *is = new std::ifstream(m_filenameRead);
    //TODO: SEEK

    return is;
}

void FileSystemEntityDataStreamProvider::endRead(upnsIStream *strm)
{
    delete strm;
}

upnsOStream *upns::FileSystemEntityDataStreamProvider::startWrite(upnsuint64 start, upnsuint64 len)
{
    //TODO: seek, overwrite
    return new std::ofstream(m_filenameWrite);
}

void FileSystemEntityDataStreamProvider::endWrite(upnsOStream *strm)
{
    assert(static_cast<std::ofstream*>(strm)->is_open());
    strm->flush();
    static_cast<std::ofstream*>(strm)->close();
    delete strm;
}

upnsuint64 FileSystemEntityDataStreamProvider::getStreamSize() const
{
    std::ifstream is(m_filenameRead);
    is.seekg(0, std::ios::beg);
    is.seekg(0, std::ios::end);
    std::stringstream::pos_type size = is.tellg();
    return size;
}

void FileSystemEntityDataStreamProvider::setStreamSize(upnsuint64 streamSize)
{
    std::ofstream os(m_filenameWrite);
    // TODO
}

LockHandle FileSystemEntityDataStreamProvider::lock()
{
    //TODO: impl
    return 0;
}

void FileSystemEntityDataStreamProvider::unlock(LockHandle)
{
    //TODO: impl
}

}
