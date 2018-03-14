/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *                2015 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "mapit/layertypes/openvdblayer.h"
#include <sstream>
#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>
#include <openvdb/io/io.h>
#include <openvdb/io/Stream.h>
#include <mapit/logging.h>

const char *FloatGridEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

void readFloatGridFromStream(std::istream &is, openvdb::FloatGrid::Ptr &grid)
{
    openvdb::initialize();
    openvdb::io::Stream strm(is);
    openvdb::GridPtrVecPtr grids(new openvdb::GridPtrVec);
    grids = strm.getGrids();
    if(grids->size() == 0 )
    {
        log_warn("no grids in file. Using empty Float Grid.");
    }
    if(grids->size() > 1 )
    {
        log_warn("multiple grids in file. Using only first Float Grid.");
    }
    openvdb::GridBase::Ptr gridBase(grids->at(0));
    openvdb::FloatGrid::Ptr gridFloat(openvdb::gridPtrCast<openvdb::FloatGrid>(gridBase));
    grid = gridFloat;
}

void writeFloatGridToStream(std::ostream &os, openvdb::FloatGrid::Ptr grid)
{
    openvdb::GridPtrVecPtr grids(new openvdb::GridPtrVec);
    grids->push_back(grid);
    openvdb::io::Stream strm(os);
    strm.write(*grids);
}


FloatGridEntitydata::FloatGridEntitydata(std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider)
    :m_streamProvider( streamProvider ),
     m_floatGrid( NULL )
{
}

const char *FloatGridEntitydata::type() const
{
    return FloatGridEntitydata::TYPENAME();
}

bool FloatGridEntitydata::hasFixedGrid() const
{
    return true;
}

bool FloatGridEntitydata::canSaveRegions() const
{
    return false;
}

FloatGridPtr FloatGridEntitydata::getData(float x1, float y1, float z1,
                                                float x2, float y2, float z2,
                                                bool clipMode,
                                                int lod)
{
    if(m_floatGrid == NULL)
    {
        mapit::istream *in = m_streamProvider->startRead();
        {
            readFloatGridFromStream( *in, m_floatGrid );
        }
        m_streamProvider->endRead(in);
    }
    return m_floatGrid;
}

int FloatGridEntitydata::setData(float x1, float y1, float z1,
                                 float x2, float y2, float z2,
                                 FloatGridPtr &data,
                                 int lod)
{
    mapit::ostream *out = m_streamProvider->startWrite();
    {
        writeFloatGridToStream( *out, data );
    }
    m_streamProvider->endWrite(out);
    return 0;
}

FloatGridPtr FloatGridEntitydata::getData(int lod)
{
    return getData(-std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                   false, lod);
}

int FloatGridEntitydata::setData(FloatGridPtr &data, int lod)
{
    return setData(-std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                   data, lod);
}

void FloatGridEntitydata::gridCellAt(float   x, float   y, float   z,
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

int FloatGridEntitydata::getEntityBoundingBox(float &x1, float &y1, float &z1,
                                              float &x2, float &y2, float &z2)
{
    openvdb::CoordBBox bbox = getData()->evalActiveVoxelBoundingBox();
    x1 = bbox.min().x();
    y1 = bbox.min().y();
    z1 = bbox.min().z();
    x2 = bbox.max().x();
    y2 = bbox.max().y();
    z2 = bbox.max().z();
    return 0;
}

mapit::istream *FloatGridEntitydata::startReadBytes(mapit::uint64_t start, mapit::uint64_t len)
{
    return m_streamProvider->startRead(start, len);
}

void FloatGridEntitydata::endRead(mapit::istream *&strm)
{
    m_streamProvider->endRead(strm);
}

mapit::ostream *FloatGridEntitydata::startWriteBytes(mapit::uint64_t start, mapit::uint64_t len)
{
    return m_streamProvider->startWrite(start, len);
}

void FloatGridEntitydata::endWrite(mapit::ostream *&strm)
{
    m_streamProvider->endWrite(strm);
}

size_t FloatGridEntitydata::size() const
{
    m_streamProvider->getStreamSize();
}

void deleteEntitydataGrid(mapit::AbstractEntitydata *ld)
{
    FloatGridEntitydata *p = dynamic_cast<FloatGridEntitydata*>(ld);
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
    *out = std::shared_ptr<mapit::AbstractEntitydata>(new FloatGridEntitydata( streamProvider ), deleteEntitydataGrid);
}
