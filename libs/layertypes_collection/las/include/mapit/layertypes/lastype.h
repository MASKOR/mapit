/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2016 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#ifndef LAZTYPE_H
#define LAZTYPE_H
#define NOMINMAX
#include <memory>
#include <mapit/entitydata.h>
#include <mapit/operators/serialization/abstractentitydataprovider.h>
#include "liblas/liblas.hpp"

#include "lasentitydatareader.h"
#include "lasentitydatawriter.h"

#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif

extern "C"
{
MODULE_EXPORT void createEntitydata(std::shared_ptr<mapit::AbstractEntitydata> *out, std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider);
}

class LASEntitydataPrivate;
class LASEntitydataReader;
class LASEntitydataWriter;

///
/// \brief The LASEntitydata class
/// Exposes Reader and Writer for LAS/LAZ data.
/// Duo to the intrfae of libLAS the interface of this Entitytype is a bit more comple than other types.
/// Instead of having easy getData/setData methods, single points can be queried directly from the data.
/// In order to be compatible with ${insert_upns_framework_name_here} you should call start(Reading|Writing)LAS before accessing the data
/// and the corresponding end-Method afterwards.
/// E.g. calling endWritingLAS while accessing a network stream will trigger the actual network connection.
/// These Methods are not reentrant or thread safe for a single instance. However it should be possible to read and write at the same time (TODO: This must be tested).
/// It should moreover be possible to write multiple entitydatas at the same time. (TODO: This must be tested when accessing same blob and isReadWriteSame)
///
class LASEntitydata : public mapit::AbstractEntitydata
{
public:
    static const char* TYPENAME();

    LASEntitydata(std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider);
    ~LASEntitydata();

    const char *type() const;
    bool        hasFixedGrid() const;
    bool        canSaveRegions() const;

    void gridCellAt(float x, float y, float z, float &x1, float &y1, float &z1, float &x2, float &y2, float &z2) const;

    std::unique_ptr<LASEntitydataReader> getReader();

    ///
    /// IMPORTANT NOTICE: Header must be fully filled here. E.g. number of point must be known in advance
    /// liblas seems to have this abstracted (with setHeader and write header methods) but this is not
    /// the case and will lead to broken las files
    /// \brief getWriter
    /// \param header
    /// \return
    ///
    std::unique_ptr<LASEntitydataWriter> getWriter(const liblas::Header &header);

    mapit::istream *startReadBytes(mapit::uint64_t start, mapit::uint64_t len);
    void endRead(mapit::istream *&strm);

    mapit::ostream *startWriteBytes(mapit::uint64_t start, mapit::uint64_t len);
    void endWrite(mapit::ostream *&strm);

    size_t size() const;

private:
    LASEntitydataPrivate *m_pimpl;
};

#endif
