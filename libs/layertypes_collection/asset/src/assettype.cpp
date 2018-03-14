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

#include "mapit/layertypes/assettype.h"
#include <mapit/logging.h>
#include <mapit/errorcodes.h>
#include "tinyply.h"

const char *AssetEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

AssetEntitydata::AssetEntitydata(std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider)
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

AssetPtr AssetEntitydata::getData(float x1, float y1, float z1,
                                                float x2, float y2, float z2,
                                                bool clipMode,
                                                int lod)
{
    if(m_asset == NULL)
    {
        mapit::istream *in = m_streamProvider->startRead();
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

int AssetEntitydata::setData(float x1, float y1, float z1,
                             float x2, float y2, float z2,
                             AssetPtr &data,
                             int lod)
{
    mapit::StatusCode s;
    mapit::ostream *out = m_streamProvider->startWrite();
    {
        m_asset = data;
        m_asset->first.write(*out, true);
    }
    m_streamProvider->endWrite(out);
    return s;
}

AssetPtr AssetEntitydata::getData(int lod)
{
    return getData(-std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                   false, lod);
}

int AssetEntitydata::setData(AssetPtr &data, int lod)
{
    return setData(-std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                   data, lod);
}

void AssetEntitydata::gridCellAt(float   x, float   y, float   z,
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

int AssetEntitydata::getEntityBoundingBox(float &x1, float &y1, float &z1,
                                              float &x2, float &y2, float &z2)
{
    //TODO
    return 1;
}

mapit::istream *AssetEntitydata::startReadBytes(mapit::uint64_t start, mapit::uint64_t len)
{
    return m_streamProvider->startRead(start, len);
}

void AssetEntitydata::endRead(mapit::istream *&strm)
{
    //This is done in destructor/shared pointer deletion
    //m_streamProvider->endRead(strm);
}

mapit::ostream *AssetEntitydata::startWriteBytes(mapit::uint64_t start, mapit::uint64_t len)
{
    return m_streamProvider->startWrite(start, len);
}

void AssetEntitydata::endWrite(mapit::ostream *&strm)
{
    m_streamProvider->endWrite(strm);
}

size_t AssetEntitydata::size() const
{
    return m_streamProvider->getStreamSize();
}

void deleteEntitydataAsset(mapit::AbstractEntitydata *ld)
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
void createEntitydata(std::shared_ptr<mapit::AbstractEntitydata> *out, std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider)
{
    *out = std::shared_ptr<mapit::AbstractEntitydata>(new AssetEntitydata( streamProvider ), deleteEntitydataAsset);
}
