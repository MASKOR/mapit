/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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

#include "mapit/layertypes/lastype.h"
#include <mapit/logging.h>
#include <mapit/errorcodes.h>

class LASEntitydataPrivate
{
public:
    std::shared_ptr<mapit::AbstractEntitydataProvider> m_streamProvider;
    LASEntitydataPrivate(std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider)
        :m_streamProvider( streamProvider ) {}
};

const char *LASEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

LASEntitydata::LASEntitydata(std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider)
    :m_pimpl(new LASEntitydataPrivate(streamProvider))
{
}

LASEntitydata::~LASEntitydata()
{
    delete m_pimpl;
}

const char* LASEntitydata::type() const
{
    return LASEntitydata::TYPENAME();
}

bool LASEntitydata::hasFixedGrid() const
{
    return false;
}

bool LASEntitydata::canSaveRegions() const
{
    return false;
}

void LASEntitydata::gridCellAt(float   x, float   y, float   z,
                                     float &x1, float &y1, float &z1,
                                     float &x2, float &y2, float &z2) const
{
    x1 = -std::numeric_limits<float>::infinity();
    y1 = -std::numeric_limits<float>::infinity();
    z1 = -std::numeric_limits<float>::infinity();
    x2 = +std::numeric_limits<float>::infinity();
    y2 = +std::numeric_limits<float>::infinity();
    z2 = +std::numeric_limits<float>::infinity();
}

std::unique_ptr<LASEntitydataReader> LASEntitydata::getReader()
{
    return std::unique_ptr<LASEntitydataReader>(new LASEntitydataReader(m_pimpl->m_streamProvider));
}

std::unique_ptr<LASEntitydataWriter> LASEntitydata::getWriter(const liblas::Header &header)
{
    return std::unique_ptr<LASEntitydataWriter>(new LASEntitydataWriter(m_pimpl->m_streamProvider, header));
}

mapit::istream *LASEntitydata::startReadBytes(mapit::uint64_t start, mapit::uint64_t len)
{
    return m_pimpl->m_streamProvider->startRead(start, len);
}

void LASEntitydata::endRead(mapit::istream *&strm)
{
    m_pimpl->m_streamProvider->endRead(strm);
}

mapit::ostream *LASEntitydata::startWriteBytes(mapit::uint64_t start, mapit::uint64_t len)
{
    return m_pimpl->m_streamProvider->startWrite(start, len);
}

void LASEntitydata::endWrite(mapit::ostream *&strm)
{
    m_pimpl->m_streamProvider->endWrite(strm);
}

size_t LASEntitydata::size() const
{
    return m_pimpl->m_streamProvider->getStreamSize();
}

//liblas::Header &LASEntitydata::GetHeader() const
//{
//    if(m_istream) return m_reader->GetHeader();
//    else if(m_ostream) return m_writer->GetHeader();
//    else log_error("Cannot get Header before starting data access (call start(Read/Write)LAS before)");
//}

//liblas::Header const& LASEntitydata::GetHeader() const
//{
//    if(m_istream) return m_reader->GetHeader();
//    else if(m_ostream) return m_writer->GetHeader();
//    else log_error("Cannot get Header before starting data access (call start(Read/Write)LAS before)");
//}

//void LASEntitydata::ReadHeader()
//{
//    if(m_istream) m_reader->ReadHeader();
//    else log_error("Cannot ReadHeader before starting data access (call startReadLAS before)");
//}

//void LASEntitydata::SetHeader(const liblas::Header &header)
//{
//    if(m_istream) m_reader->SetHeader(header);
//    else if(m_ostream) m_writer->SetHeader(header);
//    else log_error("Cannot SetHeader before starting data access (call start(Read/Write)LAS before)");
//}

//void LASEntitydata::UpdatePointCount(uint32_t count)
//{
//    if(m_ostream) return m_writer->UpdatePointCount(count);
//    else log_error("Cannot UpdatePointCount before starting data access (call startWriteLAS before)");
//}

//void LASEntitydata::WritePoint(const liblas::Point &point)
//{
//    if(m_ostream) return m_writer->WritePoint(point);
//    else log_error("Cannot GetPoint before starting data access (call startWriteLAS before)");
//}

//const liblas::Point &LASEntitydata::GetPoint() const
//{
//    if(m_istream) return m_reader->GetPoint();
//    else log_error("Cannot GetPoint before starting data access (call startReadLAS before)");
//}

//void LASEntitydata::ReadNextPoint()
//{
//    if(m_istream) return m_reader->ReadNextPoint();
//    else log_error("Cannot ReadNextPoint before starting data access (call startReadLAS before)");
//}

//const liblas::Point &LASEntitydata::ReadPointAt(std::size_t n)
//{
//    if(m_istream) return m_reader->ReadPointAt(n);
//    else log_error("Cannot ReadPointAt before starting data access (call startReadLAS before)");
//}

//void LASEntitydata::Seek(std::size_t n)
//{
//    if(m_istream) m_reader->Seek(n);
//    else log_error("Cannot Seek before starting data access (call startReadLAS before)");
//}

//void LASEntitydata::Reset()
//{
//    if(m_istream) m_reader->Reset();
//    else log_error("Cannot Reset before starting data access (call startReadLAS before)");
//}

//void LASEntitydata::SetFilters(const std::vector<liblas::FilterPtr> &filters)
//{
//    if(m_istream) m_reader->SetFilters(filters);
//    else if(m_ostream) m_writer->SetFilters(filters);
//    else log_error("Cannot SetFilters before starting data access (call start(Read/Write)LAS before)");
//}

//void LASEntitydata::SetTransforms(const std::vector<liblas::TransformPtr> &transforms)
//{
//    if(m_istream) m_reader->SetTransforms(transforms);
//    else if(m_ostream) m_writer->SetTransforms(transforms);
//    else log_error("Cannot SetTransforms before starting data access (call start(Read/Write)LAS before)");
//}

//std::vector<liblas::TransformPtr> LASEntitydata::GetTransforms() const
//{
//    if(m_istream) return m_reader->GetTransforms();
//    else if(m_ostream) return m_writer->GetTransforms();
//    else log_error("Cannot GetTransforms before starting data access (call start(Read/Write)LAS before)");
//}

//std::vector<liblas::FilterPtr> LASEntitydata::GetFilters() const
//{
//    if(m_istream) return m_reader->GetFilters();
//    else if(m_ostream) return m_writer->GetFilters();
//    else log_error("Cannot GetFilters before starting data access (call start(Read/Write)LAS before)");
//}

//void LASEntitydata::WriteHeader()
//{
//    if(m_ostream) m_writer->WriteHeader();
//    else log_error("Cannot WriteHeader before starting data access (call startWriteLAS before)");
//}


//void LASEntitydata::startWritingLAS(liblas::Header const& header)
//{
//    if(m_ostream != nullptr)
//    {
//        log_error("Started writing while already writing.");
//        return;
//    }
//    liblas::WriterFactory f;
//    m_ostream = m_pimpl->m_streamProvider->startWrite();
//    m_writer = new liblas::Writer(f.CreateWithStream(*m_ostream, header));
//}

//void LASEntitydata::endWritingLAS()
//{
//    if(m_ostream == nullptr)
//    {
//        log_error("End reading, but never started reading before.");
//        return;
//    }
//    m_pimpl->m_streamProvider->endWrite(m_ostream);
//    m_ostream = nullptr;
//}

void deleteEntitydataLAS(mapit::AbstractEntitydata *ld)
{
    LASEntitydata *p = dynamic_cast<LASEntitydata*>(ld);
    if(p)
    {
        delete p;
    }
    else
    {
        log_error("Wrong entitytype");
    }
}
void createEntitydata(std::shared_ptr<mapit::AbstractEntitydata> *out, std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider)
{
    *out = std::shared_ptr<mapit::AbstractEntitydata>(new LASEntitydata( streamProvider ), deleteEntitydataLAS);
}
