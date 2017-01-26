#include "fs_entitydatastreamprovider.h"
#include <fstream>
#include <string>
#include <sstream>
#include <assert.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <upns_logging.h>
#define FILEMODE S_IRWXU | S_IRGRP | S_IROTH

namespace upns {

struct FileSystemReadWriteHandle {
    int filedescriptor;
    size_t length;
    __off_t offset;
};

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

void *mapFile(const upnsuint64 &len, const upnsuint64 &start, upnsString &filename, int &handle, int flag, int privShared, int openflag)
{
    if ((handle = open(filename.c_str(), openflag | O_CREAT, FILEMODE)) < 0)
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

void *FileSystemEntitydataStreamProvider::startReadPointer(ReadWriteHandle &handle, upnsuint64 start, upnsuint64 len)
{
    FileSystemReadWriteHandle *fsHandle = new FileSystemReadWriteHandle;
    handle = static_cast<ReadWriteHandle>(fsHandle);
    fsHandle->length = len;
    fsHandle->offset = start;
    return mapFile(len?len:getStreamSize(), start, m_filenameRead, fsHandle->filedescriptor, PROT_READ, MAP_SHARED, O_RDONLY);
}

void FileSystemEntitydataStreamProvider::endReadPointer(void* ptr, ReadWriteHandle &handle)
{
    FileSystemReadWriteHandle *fsHandle(static_cast<FileSystemReadWriteHandle*>(handle));
    if (munmap(ptr, fsHandle->length) == -1)
    {
        log_error("Error un-mmapping the file");
        close(fsHandle->filedescriptor);
    }
    delete fsHandle;
}

void *FileSystemEntitydataStreamProvider::startWritePointer(ReadWriteHandle &handle, upnsuint64 start, upnsuint64 len)
{
    FileSystemReadWriteHandle *fsHandle = new FileSystemReadWriteHandle;
    handle = static_cast<ReadWriteHandle>(fsHandle);
    fsHandle->length = len;
    fsHandle->offset = start;
    return mapFile(len?len:getStreamSize(), start, m_filenameWrite, fsHandle->filedescriptor, PROT_WRITE, MAP_SHARED, O_WRONLY);
}

void FileSystemEntitydataStreamProvider::endWritePointer(void* ptr, ReadWriteHandle &handle)
{
    FileSystemReadWriteHandle *fsHandle(static_cast<FileSystemReadWriteHandle*>(handle));
    if (munmap(ptr, fsHandle->length) == -1)
    {
        log_error("Error un-mmapping the file");
        close(fsHandle->filedescriptor);
    }
    delete fsHandle;
}

upnsString FileSystemEntitydataStreamProvider::startReadFile(ReadWriteHandle &handle)
{
    return m_filenameRead;
}

void FileSystemEntitydataStreamProvider::endReadFile(ReadWriteHandle &handle)
{

}

upnsString FileSystemEntitydataStreamProvider::startWriteFile(ReadWriteHandle &handle)
{
    return m_filenameWrite;
}

void FileSystemEntitydataStreamProvider::endWriteFile(ReadWriteHandle &handle)
{

}

AbstractEntitydataStreamProvider::ReadWriteType FileSystemEntitydataStreamProvider::preferredReadType()
{
    return ReadWriteFile;
}

AbstractEntitydataStreamProvider::ReadWriteType FileSystemEntitydataStreamProvider::preferredWriteType()
{
    return ReadWriteFile;
}

}
