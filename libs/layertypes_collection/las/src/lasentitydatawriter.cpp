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

#include "upns/layertypes/lasentitydatawriter.h"
#include <upns/logging.h>
#include <memory>

class LASEntitydataWriterPrivate
{
public:
    LASEntitydataWriterPrivate(std::shared_ptr<AbstractEntitydataProvider> prov, const liblas::Header& header)
        : m_streamProvider(prov)
    {
        liblas::WriterFactory f;
        //m_tmpFile.open(m_streamProvider->startWriteFile(m_readWriteHandleTmp));
        //m_writer = new liblas::Writer(f.CreateWithStream(m_tmpFile, header));
        //m_ostream = &m_tmpFile;
        m_ostream = m_streamProvider->startWrite();
        m_writer = new liblas::Writer(f.CreateWithStream(*m_ostream, header));
        m_writer->SetHeader(header);
        m_writer->WriteHeader();
    }
    ~LASEntitydataWriterPrivate()
    {
        if(m_ostream == nullptr)
        {
            log_error("End writing, but never started writing before.");
            return;
        }
        //m_streamProvider->endWriteFile(m_readWriteHandleTmp);
        //m_writer->SetHeader(*m_header);
        //m_writer->WriteHeader();
        m_streamProvider->endWrite(m_ostream);
        m_ostream = nullptr;
    }
    std::shared_ptr<AbstractEntitydataProvider> m_streamProvider;
    liblas::Writer *m_writer;
    upnsOStream    *m_ostream;
    // a copy of the header is needed so it does not need to be set multiple times from outside
    // header must be Set at beginning of writing (for point format) and at the end (to communicate point size).
    //std::shared_ptr<liblas::Header> m_header;
//    ReadWriteHandle m_readWriteHandleTmp;
//    std::ofstream   m_tmpFile;
};

const liblas::Header &LASEntitydataWriter::GetHeader() const
{
    return m_pimpl->m_writer->GetHeader();
}

//void LASEntitydataWriter::SetHeader(const liblas::Header &header)
//{
//    m_pimpl->m_writer->SetHeader(header);
//}

bool LASEntitydataWriter::WritePoint(const liblas::Point &point)
{
    return m_pimpl->m_writer->WritePoint(point);
}

//void LASEntitydataWriter::WriteHeader()
//{
//    m_pimpl->m_writer->WriteHeader();
//}

void LASEntitydataWriter::SetFilters(const std::vector<liblas::FilterPtr> &filters)
{
    m_pimpl->m_writer->SetFilters(filters);
}

std::vector<liblas::FilterPtr> LASEntitydataWriter::GetFilters() const
{
    return m_pimpl->m_writer->GetFilters();
}

void LASEntitydataWriter::SetTransforms(const std::vector<liblas::TransformPtr> &transforms)
{
    m_pimpl->m_writer->SetTransforms(transforms);
}

std::vector<liblas::TransformPtr> LASEntitydataWriter::GetTransforms() const
{
    return m_pimpl->m_writer->GetTransforms();
}

LASEntitydataWriter::LASEntitydataWriter(std::shared_ptr<AbstractEntitydataProvider> prov, const liblas::Header &header)
    :m_pimpl(new LASEntitydataWriterPrivate(prov, header))
{
}

LASEntitydataWriter::~LASEntitydataWriter()
{
    delete m_pimpl;
}
