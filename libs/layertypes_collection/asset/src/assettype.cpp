#include "assettype.h"
#include "upns_logging.h"
#include "upns_errorcodes.h"
#include "tinyply.h"

const char *AssetEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

AssetEntitydata::AssetEntitydata(upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider)
    :m_streamProvider( streamProvider ),
     m_asset( NULL )
{
}

const char *AssetEntitydata::type() const
{
    return AssetEntitydata::TYPENAME();
}

bool AssetEntitydata::hasFixedGrid() const
{
    return false;
}

bool AssetEntitydata::canSaveRegions() const
{
    return false;
}

upnsAssetPtr AssetEntitydata::getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                                upnsReal x2, upnsReal y2, upnsReal z2,
                                                bool clipMode,
                                                int lod)
{
    if(m_asset == NULL)
    {
        upnsIStream *in = m_streamProvider->startRead();
        {
            m_asset = upnsAssetPtr(new tinyply::PlyFile);
            m_asset->read(*in);
        }
        m_streamProvider->endRead(in);
    }
    return m_asset;
}

int AssetEntitydata::setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                 upnsReal x2, upnsReal y2, upnsReal z2,
                                 upnsAssetPtr &data,
                                 int lod)
{
    StatusCode s;
    upnsOStream *out = m_streamProvider->startWrite();
    {
        m_asset = data;
        m_asset->write(*out, true);
    }
    m_streamProvider->endWrite(out);
    return s;
}

upnsAssetPtr AssetEntitydata::getData(int lod)
{
    return getData(-std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                   false, lod);
}

int AssetEntitydata::setData(upnsAssetPtr &data, int lod)
{
    return setData(-std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                   data, lod);
}

void AssetEntitydata::gridCellAt(upnsReal   x, upnsReal   y, upnsReal   z,
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

int AssetEntitydata::getEntityBoundingBox(upnsReal &x1, upnsReal &y1, upnsReal &z1,
                                              upnsReal &x2, upnsReal &y2, upnsReal &z2)
{
    //TODO
    return 0;
}

upnsIStream *AssetEntitydata::startReadBytes(upnsuint64 start, upnsuint64 len)
{
    return m_streamProvider->startRead(start, len);
}

void AssetEntitydata::endRead(upnsIStream *strm)
{
    m_streamProvider->endRead(strm);
}

upnsOStream *AssetEntitydata::startWriteBytes(upnsuint64 start, upnsuint64 len)
{
    return m_streamProvider->startWrite(start, len);
}

void AssetEntitydata::endWrite(upnsOStream *strm)
{
    m_streamProvider->endWrite(strm);
}

size_t AssetEntitydata::size() const
{
    m_streamProvider->getStreamSize();
}

void deleteEntitydata(AbstractEntitydata *ld)
{
    AssetEntitydata *p = static_cast<AssetEntitydata*>(ld);
    delete p;
}
void createEntitydata(upnsSharedPointer<AbstractEntitydata> *out, upnsSharedPointer<AbstractEntitydataStreamProvider> streamProvider)
{
    *out = upnsSharedPointer<AbstractEntitydata>(new AssetEntitydata( streamProvider ), deleteEntitydata);
}

