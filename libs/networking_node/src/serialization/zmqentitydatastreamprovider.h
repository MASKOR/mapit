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

#ifndef ZMQENTITYDATASTREAMPROVIDER_H
#define ZMQENTITYDATASTREAMPROVIDER_H

#include <mapit/typedefs.h>
#include <mapit/versioning/repository.h>
#include "zmqprotobufnode.h"
#include <mapit/operators/serialization/abstractentitydataprovider.h>
#include <mutex>

namespace mapit
{

/**
 * @brief ZmqEntitydataStreamProvider sends binary data over network
 */

class ZmqEntitydataStreamProvider : public AbstractEntitydataProvider
{
    // AbstractEntitydataProvider interface
public:
    ZmqEntitydataStreamProvider(std::string workspaceName, std::string pathOrOid, ZmqProtobufNode *node, std::mutex *requestMutex);
    bool isCached();
    bool isReadWriteSame();
    mapit::istream *startRead(mapit::uint64_t start, mapit::uint64_t length);
    void endRead(mapit::istream *&strm);
    mapit::ostream *startWrite(mapit::uint64_t start, mapit::uint64_t len);
    void endWrite(mapit::ostream *&strm);
    mapit::uint64_t getStreamSize() const;
    void setStreamSize(mapit::uint64_t entitylength);
    LockHandle lock();
    void unlock(LockHandle);

    const void *startReadPointer(ReadWriteHandle &handle, mapit::uint64_t start, mapit::uint64_t len);
    void endReadPointer(const void *ptr, ReadWriteHandle &handle);
    void *startWritePointer(ReadWriteHandle &handle, mapit::uint64_t start, mapit::uint64_t len);
    void endWritePointer(void *ptr, ReadWriteHandle &handle);
    char *startRead(mapit::uint64_t start, mapit::uint64_t length, mapit::uint64_t &outLength);
    //void endRead();
    void endWrite(const char *memory, const mapit::uint64_t &length, const mapit::uint64_t &offset);

    std::string startReadFile(ReadWriteHandle &handle);
    void endReadFile(ReadWriteHandle &handle);
    std::string startWriteFile(ReadWriteHandle &handle);
    void endWriteFile(ReadWriteHandle &handle);

    ReadWriteType preferredReadType();
    ReadWriteType preferredWriteType();
private:
    std::string m_workspaceName;
    std::string m_pathOrOid;
    ZmqProtobufNode *m_node;
    mutable mapit::uint64_t m_entityLength;
    std::mutex *m_requestMutex;
};

}
#endif // ZMQENTITYDATASTREAMPROVIDER_H
