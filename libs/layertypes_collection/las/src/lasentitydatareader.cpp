#include "upns/layertypes/lasentitydatareader.h"
#include <upns/logging.h>

class LASEntitydataReaderPrivate
{
public:
    LASEntitydataReaderPrivate(std::shared_ptr<AbstractEntitydataProvider> prov)
        :m_streamProvider(prov)
    {
        liblas::ReaderFactory f;
        m_istream = m_streamProvider->startRead();
        m_reader = new liblas::Reader(f.CreateWithStream(*m_istream));
    }
    ~LASEntitydataReaderPrivate()
    {
        if(m_istream == nullptr)
        {
            log_error("End reading, but never started reading before.");
            return;
        }
        m_streamProvider->endRead(m_istream);
        m_istream = nullptr;
    }
    std::shared_ptr<AbstractEntitydataProvider> m_streamProvider;
    liblas::Reader *m_reader;
    upnsIStream    *m_istream;
};

const liblas::Point &LASEntitydataReader::GetPoint() const
{
    return m_pimpl->m_reader->GetPoint();
}

bool LASEntitydataReader::ReadNextPoint()
{
    return m_pimpl->m_reader->ReadNextPoint();
}

bool LASEntitydataReader::ReadPointAt(std::size_t n)
{
    return m_pimpl->m_reader->ReadPointAt(n);
}

void LASEntitydataReader::Reset()
{
    m_pimpl->m_reader->Reset();
}

bool LASEntitydataReader::Seek(std::size_t n)
{
    return m_pimpl->m_reader->Seek(n);
}

const liblas::Point &LASEntitydataReader::operator[](std::size_t n)
{
    return (*m_pimpl->m_reader)[n];
}

void LASEntitydataReader::SetFilters(const std::vector<liblas::FilterPtr> &filters)
{
    m_pimpl->m_reader->SetFilters(filters);
}

std::vector<liblas::FilterPtr> LASEntitydataReader::GetFilters() const
{
    return m_pimpl->m_reader->GetFilters();
}

void LASEntitydataReader::SetTransforms(const std::vector<liblas::TransformPtr> &transforms)
{
    m_pimpl->m_reader->SetTransforms(transforms);
}

std::vector<liblas::TransformPtr> LASEntitydataReader::GetTransforms() const
{
    return m_pimpl->m_reader->GetTransforms();
}

liblas::Reader *LASEntitydataReader::getReaderRaw()
{
    return m_pimpl->m_reader;
}

LASEntitydataReader::LASEntitydataReader(std::shared_ptr<AbstractEntitydataProvider> prov)
    :m_pimpl(new LASEntitydataReaderPrivate(prov))
{
}
