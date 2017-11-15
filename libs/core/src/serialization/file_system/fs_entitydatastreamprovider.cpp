#include "fs_entitydatastreamprovider.h"
#include <fstream>
#include <string>
#include <sstream>
#include <assert.h>
#include <boost/iostreams/device/mapped_file.hpp>

//#ifdef _WIN32
//# include <io.h>
//# include <windows.h>
//# define memmap_open                    _open
//# define memmap_close(fd)               _close(fd)
//# define memmap_lseek(fd,offset,origin) _lseek(fd,offset,origin)
//#else
//# include <sys/mman.h>
//# define memmap_open                    open
//# define memmap_close(fd)               close(fd)
//# define memmap_lseek(fd,offset,origin) lseek(fd,offset,origin)
//#endif

////#include <sys/mman.h>
//#include <sys/stat.h>
//#include <sys/types.h>
//#include <fcntl.h>
////#include <unistd.h>

#include <upns/logging.h>
#define FILEMODE S_IRWXU | S_IRGRP | S_IROTH

namespace upns {

struct FileSystemReadWriteHandle {
    boost::iostreams::mapped_file_source file;
    size_t length;
    size_t offset;
};
struct FileSystemReadWritePointerHandle {
    boost::iostreams::mapped_file file;
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
    std::ifstream *is = new std::ifstream(m_filenameRead, std::ifstream::in | std::ios_base::binary);
    is->seekg(start, std::ios::beg);
    return is;
}

void FileSystemEntitydataStreamProvider::endRead(upnsIStream *&strm)
{
    if(strm != nullptr)
    {
        delete strm;
        strm = nullptr;
    }
    else
    {
        //TODO: this might be a bug with shared objects, shared pointers and casting between abstractEntityData. Valgrind.
        log_warn("do not end Read twice!");
    }
}

upnsOStream *upns::FileSystemEntitydataStreamProvider::startWrite(upnsuint64 start, upnsuint64 len)
{
    //TODO: seek, overwrite
    if( m_filenameWrite.empty() )
    {
        log_error("writing not allowed");
        return nullptr;
    }
    std::ofstream *ofstr(new std::ofstream(m_filenameWrite, std::ios::out | std::ios::binary));
    assert(ofstr->is_open());
    return ofstr;
}

void FileSystemEntitydataStreamProvider::endWrite(upnsOStream *&strm)
{
    assert(strm);
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
    if( m_filenameWrite.empty() )
    {
        log_error("writing not allowed");
        return;
    }
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

const void *FileSystemEntitydataStreamProvider::startReadPointer(ReadWriteHandle &handle, upnsuint64 start, upnsuint64 len)
{
    FileSystemReadWriteHandle *fsHandle = new FileSystemReadWriteHandle;
    handle = static_cast<ReadWriteHandle>(fsHandle);
    fsHandle->length = len;
    fsHandle->offset = start;
    fsHandle->file.open(m_filenameRead.c_str(), len?len:getStreamSize(), start);
    if(fsHandle->file.is_open())
    {
        // Get pointer to the data
        return reinterpret_cast<const void*>(fsHandle->file.data());
    }
    else
    {
        log_error("Error opening file as mapped memory file: " + m_filenameRead);
        return nullptr;
    }
}

void FileSystemEntitydataStreamProvider::endReadPointer(const void *ptr, ReadWriteHandle &handle)
{
    assert(ptr);
    FileSystemReadWriteHandle *fsHandle(static_cast<FileSystemReadWriteHandle*>(handle));
    assert(fsHandle->file.is_open());
    // Remember to unmap the file
    fsHandle->file.close();
    if (fsHandle->file.is_open())
    {
        log_error("Error un-mmapping the file");
    }
    delete fsHandle;
}

void *FileSystemEntitydataStreamProvider::startWritePointer(ReadWriteHandle &handle, upnsuint64 start, upnsuint64 len)
{
    if( m_filenameWrite.empty() ) {
        log_error("writing not allowed");
        return nullptr;
    }
    FileSystemReadWritePointerHandle *fsHandle = new FileSystemReadWritePointerHandle;
    handle = static_cast<ReadWriteHandle>(fsHandle);
    fsHandle->file.open(m_filenameWrite, boost::iostreams::mapped_file::priv, len?len:getStreamSize(), start);
    return fsHandle->file.data();
//    boost::iostreams::mapped_file_params params;
//    params.path          = m_filenameWrite;
//    params.new_file_size = len?len:getStreamSize();
//    params.flags         = boost::iostreams::mapped_file::mapmode::readwrite;

//    boost::iostreams::stream<boost::iostreams::mapped_file_sink> out(params);
}

void FileSystemEntitydataStreamProvider::endWritePointer(void* ptr, ReadWriteHandle &handle)
{
    assert(ptr);
    FileSystemReadWritePointerHandle *fsHandle(static_cast<FileSystemReadWritePointerHandle*>(handle));
    fsHandle->file.close();
    if (fsHandle->file.is_open())
    {
        log_error("Error un-mmapping the file");
    }
    delete fsHandle;
}

std::string FileSystemEntitydataStreamProvider::startReadFile(ReadWriteHandle &handle)
{
    if( m_filenameRead.empty() )
    {
        log_error("reading not allowed");
        return "";
    }
    return m_filenameRead;
}

void FileSystemEntitydataStreamProvider::endReadFile(ReadWriteHandle &handle)
{

}

std::string FileSystemEntitydataStreamProvider::startWriteFile(ReadWriteHandle &handle)
{
    if( m_filenameWrite.empty() )
    {
        log_error("writing not allowed");
        return "";
    }
    return m_filenameWrite;
}

void FileSystemEntitydataStreamProvider::endWriteFile(ReadWriteHandle &handle)
{
    //Nothing to do here, file has been written
}

AbstractEntitydataProvider::ReadWriteType FileSystemEntitydataStreamProvider::preferredReadType()
{
    return ReadWriteFile;
}

AbstractEntitydataProvider::ReadWriteType FileSystemEntitydataStreamProvider::preferredWriteType()
{
    return ReadWriteFile;
}

}
