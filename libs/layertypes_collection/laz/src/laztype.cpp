#include "laztype.h"
#include "upns_logging.h"
#include "upns_errorcodes.h"

const char *LASzipEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

LASzipEntitydata::LASzipEntitydata(upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider)
    :m_streamProvider( streamProvider ),
     m_e( NULL )
{
}

const char* LASzipEntitydata::type() const
{
    return LASzipEntitydata::TYPENAME();
}

bool LASzipEntitydata::hasFixedGrid() const
{
    return false;
}

bool LASzipEntitydata::canSaveRegions() const
{
    return false;
}

upnsLASzipPtr LASzipEntitydata::getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                                upnsReal x2, upnsReal y2, upnsReal z2,
                                                bool clipMode,
                                                int lod)
{
    if(m_e == nullptr)
    {
        m_e = upnsLASzipPtr(new LASzip);
        LASunzipper unzipper;
        switch(m_streamProvider->preferredReadType())
        {
        case AbstractEntitydataStreamProvider::ReadWriteFile:
        {
            ReadWriteHandle handle;
            upnsString filename = m_streamProvider->startReadFile(handle);
            FILE *file = fopen(filename.c_str(), "rb");
            unzipper.open(file, m_e.get());
            fclose(file);
            m_streamProvider->endReadFile(handle);
            break;
        }
        case AbstractEntitydataStreamProvider::ReadWriteStream:
        case AbstractEntitydataStreamProvider::ReadWritePointer:
        {
            upnsIStream *in = m_streamProvider->startRead();
            unzipper.open(*in, m_e.get());
            m_streamProvider->endRead(in);
            break;
        }
        }
    }
    return m_e;
}

int LASzipEntitydata::setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                 upnsReal x2, upnsReal y2, upnsReal z2,
                                 upnsLASzipPtr &data,
                                 int lod)
{
    StatusCode s = UPNS_STATUS_OK;
    m_e = data;
    LASzipper zipper;
    switch(m_streamProvider->preferredReadType())
    {
    case AbstractEntitydataStreamProvider::ReadWriteFile:
    {
        ReadWriteHandle handle;
        upnsString filename = m_streamProvider->startWriteFile(handle);
        FILE *file = fopen(filename.c_str(), "wb");
        zipper.open(file, m_e.get());
        zipper.write(); //TODO: MAYBE USE LIBLAS?
        fclose(file);
        m_streamProvider->endReadFile(handle);
        break;
    }
    case AbstractEntitydataStreamProvider::ReadWriteStream:
    case AbstractEntitydataStreamProvider::ReadWritePointer:
    {
        upnsIStream *in = m_streamProvider->startRead();
        unzipper.open(*in, m_e.get());
        m_streamProvider->endRead(in);
        break;
    }
    }

    upnsOStream *out = m_streamProvider->startWrite();
    {
        m_asset = data;
        m_asset->write(*out, true);
    }
    m_streamProvider->endWrite(out);
    return s;
}

upnsLASzipPtr LASzipEntitydata::getData(int lod)
{
    return getData(-std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                   false, lod);
}

int LASzipEntitydata::setData(upnsLASzipPtr &data, int lod)
{
    return setData(-std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                   data, lod);
}

void LASzipEntitydata::gridCellAt(upnsReal   x, upnsReal   y, upnsReal   z,
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

int LASzipEntitydata::getEntityBoundingBox(upnsReal &x1, upnsReal &y1, upnsReal &z1,
                                              upnsReal &x2, upnsReal &y2, upnsReal &z2)
{
    //TODO
    return 0;
}

upnsIStream *LASzipEntitydata::startReadBytes(upnsuint64 start, upnsuint64 len)
{
    return m_streamProvider->startRead(start, len);
}

void LASzipEntitydata::endRead(upnsIStream *strm)
{
    m_streamProvider->endRead(strm);
}

upnsOStream *LASzipEntitydata::startWriteBytes(upnsuint64 start, upnsuint64 len)
{
    return m_streamProvider->startWrite(start, len);
}

void LASzipEntitydata::endWrite(upnsOStream *strm)
{
    m_streamProvider->endWrite(strm);
}

size_t LASzipEntitydata::size() const
{
    m_streamProvider->getStreamSize();
}

void deleteEntitydata(AbstractEntitydata *ld)
{
    LASzipEntitydata *p = static_cast<LASzipEntitydata*>(ld);
    delete p;
}
void createEntitydata(upnsSharedPointer<AbstractEntitydata> *out, upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider)
{
    *out = upnsSharedPointer<AbstractEntitydata>(new LASzipEntitydata( streamProvider ), deleteEntitydata);
}

