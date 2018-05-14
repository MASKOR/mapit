/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#include <mapit/logging.h>
#define FILEMODE S_IRWXU | S_IRGRP | S_IROTH

namespace mapit {

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

mapit::istream* mapit::FileSystemEntitydataStreamProvider::startRead(mapit::uint64_t start, mapit::uint64_t len)
{
    std::ifstream *is = new std::ifstream(m_filenameRead, std::ifstream::in | std::ios_base::binary);
    is->seekg(start, std::ios::beg);
    return is;
}

void FileSystemEntitydataStreamProvider::endRead(mapit::istream *&strm)
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

mapit::ostream *mapit::FileSystemEntitydataStreamProvider::startWrite(mapit::uint64_t start, mapit::uint64_t len)
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

void FileSystemEntitydataStreamProvider::endWrite(mapit::ostream *&strm)
{
    assert(strm);
    assert(static_cast<std::ofstream*>(strm)->is_open());
//    strm->flush(); // removed since this is slow and not needed
    static_cast<std::ofstream*>(strm)->close();
    delete strm;
}

mapit::uint64_t FileSystemEntitydataStreamProvider::getStreamSize() const
{
    std::ifstream is(m_filenameRead);
    is.seekg(0, std::ios::beg);
    is.seekg(0, std::ios::end);
    std::stringstream::pos_type size = is.tellg();
    return size;
}

void FileSystemEntitydataStreamProvider::setStreamSize(mapit::uint64_t streamSize)
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

const void *FileSystemEntitydataStreamProvider::startReadPointer(ReadWriteHandle &handle, mapit::uint64_t start, mapit::uint64_t len)
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

void *FileSystemEntitydataStreamProvider::startWritePointer(ReadWriteHandle &handle, mapit::uint64_t start, mapit::uint64_t len)
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
