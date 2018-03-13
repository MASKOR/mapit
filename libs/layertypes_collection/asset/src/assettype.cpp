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

#include "upns/layertypes/assettype.h"
#include <upns/logging.h>
#include <upns/errorcodes.h>
#include "tinyply.h"

const char *AssetEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

AssetEntitydata::AssetEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
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

AssetPtr AssetEntitydata::getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                                upnsReal x2, upnsReal y2, upnsReal z2,
                                                bool clipMode,
                                                int lod)
{
    if(m_asset == NULL)
    {
        upnsIStream *in = m_streamProvider->startRead();
        {
            m_asset = AssetPtr(new AssetDataPair( tinyply::PlyFile(*in),in), [this](AssetDataPair* ply)
            {
                //TODO: expose custom class for tinyply
                m_streamProvider->endRead(ply->second);
            });
            //m_asset->first.read(*in);
        }
        //m_streamProvider->endRead(in);
    }
    return m_asset;
}

int AssetEntitydata::setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                 upnsReal x2, upnsReal y2, upnsReal z2,
                                 AssetPtr &data,
                                 int lod)
{
    StatusCode s;
    upnsOStream *out = m_streamProvider->startWrite();
    {
        m_asset = data;
        m_asset->first.write(*out, true);
    }
    m_streamProvider->endWrite(out);
    return s;
}

AssetPtr AssetEntitydata::getData(int lod)
{
    return getData(-std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                   -std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                    std::numeric_limits<upnsReal>::infinity(),
                   false, lod);
}

int AssetEntitydata::setData(AssetPtr &data, int lod)
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
    return 1;
}

upnsIStream *AssetEntitydata::startReadBytes(upnsuint64 start, upnsuint64 len)
{
    return m_streamProvider->startRead(start, len);
}

void AssetEntitydata::endRead(upnsIStream *&strm)
{
    //This is done in destructor/shared pointer deletion
    //m_streamProvider->endRead(strm);
}

upnsOStream *AssetEntitydata::startWriteBytes(upnsuint64 start, upnsuint64 len)
{
    return m_streamProvider->startWrite(start, len);
}

void AssetEntitydata::endWrite(upnsOStream *&strm)
{
    m_streamProvider->endWrite(strm);
}

size_t AssetEntitydata::size() const
{
    return m_streamProvider->getStreamSize();
}

void deleteEntitydataAsset(AbstractEntitydata *ld)
{
    AssetEntitydata *p = dynamic_cast<AssetEntitydata*>(ld);
    if(p)
    {
        delete p;
    }
    else
    {
        log_error("Wrong entitytype");
    }
}
void createEntitydata(std::shared_ptr<AbstractEntitydata> *out, std::shared_ptr<AbstractEntitydataProvider> streamProvider)
{
    *out = std::shared_ptr<AbstractEntitydata>(new AssetEntitydata( streamProvider ), deleteEntitydataAsset);
}

