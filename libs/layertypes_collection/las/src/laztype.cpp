#include "upns/layertypes/lastype.h"
#include <upns/logging.h>
#include <upns/errorcodes.h>

class LASEntitydataPrivate
{
public:
    upnsSharedPointer<AbstractEntitydataProvider> m_streamProvider;
    LASEntitydataPrivate(upnsSharedPointer<AbstractEntitydataProvider> streamProvider)
        :m_streamProvider( streamProvider ) {}
};

const char *LASEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

LASEntitydata::LASEntitydata(upnsSharedPointer<AbstractEntitydataProvider> streamProvider)
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

void LASEntitydata::gridCellAt(upnsReal   x, upnsReal   y, upnsReal   z,
                                     upnsReal &x1, upnsReal &y1, upnsReal &z1,
                                     upnsReal &x2, upnsReal &y2, upnsReal &z2) const
{
    x1 = -std::numeric_limits<upnsReal>::infinity();
    y1 = -std::numeric_limits<upnsReal>::infinity();
    z1 = -std::numeric_limits<upnsReal>::infinity();
    x2 = +std::numeric_limits<upnsReal>::infinity();
    y2 = +std::numeric_limits<upnsReal>::infinity();
    z2 = +std::numeric_limits<upnsReal>::infinity();
}

std::unique_ptr<LASEntitydataReader> LASEntitydata::getReader()
{
    return std::unique_ptr<LASEntitydataReader>(new LASEntitydataReader(m_pimpl->m_streamProvider));
}

std::unique_ptr<LASEntitydataWriter> LASEntitydata::getWriter(liblas::Header const& header)
{
    return std::unique_ptr<LASEntitydataWriter>(new LASEntitydataWriter(m_pimpl->m_streamProvider, header));
}

upnsIStream *LASEntitydata::startReadBytes(upnsuint64 start, upnsuint64 len)
{
    return m_pimpl->m_streamProvider->startRead(start, len);
}

void LASEntitydata::endRead(upnsIStream *strm)
{
    m_pimpl->m_streamProvider->endRead(strm);
}

upnsOStream *LASEntitydata::startWriteBytes(upnsuint64 start, upnsuint64 len)
{
    return m_pimpl->m_streamProvider->startWrite(start, len);
}

void LASEntitydata::endWrite(upnsOStream *strm)
{
    m_pimpl->m_streamProvider->endWrite(strm);
}

size_t LASEntitydata::size() const
{
    m_pimpl->m_streamProvider->getStreamSize();
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

void deleteEntitydata(AbstractEntitydata *ld)
{
    LASEntitydata *p = static_cast<LASEntitydata*>(ld);
    delete p;
}
void createEntitydata(upnsSharedPointer<AbstractEntitydata> *out, upnsSharedPointer<AbstractEntitydataProvider> streamProvider)
{
    *out = upnsSharedPointer<AbstractEntitydata>(new LASEntitydata( streamProvider ), deleteEntitydata);
}

