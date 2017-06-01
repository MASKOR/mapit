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
