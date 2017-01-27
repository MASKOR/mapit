#include "lasentitydatawriter.h"
#include "upns_logging.h"

class LASEntitydataWriterPrivate
{
public:
    LASEntitydataWriterPrivate(upnsSharedPointer<AbstractEntitydataStreamProvider> prov, liblas::Header const& header)
        :m_streamProvider(prov)
    {
        liblas::WriterFactory f;
        m_ostream = m_streamProvider->startWrite();
        m_writer = new liblas::Writer(f.CreateWithStream(*m_ostream, header));
    }
    ~LASEntitydataWriterPrivate()
    {
        if(m_ostream == nullptr)
        {
            log_error("End writing, but never started writing before.");
            return;
        }
        m_streamProvider->endWrite(m_ostream);
        m_ostream = nullptr;
    }
    upnsSharedPointer<AbstractEntitydataStreamProvider> m_streamProvider;
    liblas::Writer *m_writer;
    upnsOStream    *m_ostream;
};

const liblas::Header &LASEntitydataWriter::GetHeader() const
{
    return m_pimpl->m_writer->GetHeader();
}

void LASEntitydataWriter::SetHeader(const liblas::Header &header)
{
    m_pimpl->m_writer->SetHeader(header);
}

bool LASEntitydataWriter::WritePoint(const liblas::Point &point)
{
    return m_pimpl->m_writer->WritePoint(point);
}

void LASEntitydataWriter::WriteHeader()
{
    m_pimpl->m_writer->WriteHeader();
}

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

LASEntitydataWriter::LASEntitydataWriter(std::shared_ptr<AbstractEntitydataStreamProvider> prov, liblas::Header const& header)
    :m_pimpl(new LASEntitydataWriterPrivate(prov, header))
{
}
