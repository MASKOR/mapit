/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2016 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "mapit/layertypes/grid2dlayer.h"
#include <mapit/logging.h>
#include <sstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
//#include <pcl/compression/octree_pointcloud_compression.h>
//#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h> // for bb
#include <mapit/msgs/datastructs.pb.h>
#include <iostream>
#include <fstream>

const char *Grid2DEntitydata::TYPENAME()
{
    return PROJECT_NAME;
}

Grid2DEntitydata::Grid2DEntitydata(std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider)
    :m_streamProvider( streamProvider ),
     m_Grid2D( )
{
}

const char *Grid2DEntitydata::type() const
{
    return Grid2DEntitydata::TYPENAME();
}
/**
bool Grid2DEntitydata::hasFixedGrid() const
{
    return false;
}

bool Grid2DEntitydata::canSaveRegions() const
{
    return false;
}
*/



 mapit::entitytypes::Grid2DType Grid2DEntitydata::getData(float x1, float y1, float z1, float x2, float y2, float z2, bool clipMode, int lod)
{

    if(m_Grid2D == NULL)
    {
        m_Grid2D = mapit::entitytypes::Grid2DType(new ::mapit::msgs::Grid2D);
        mapit::ReadWriteHandle handle;
        std::string filename = m_streamProvider->startReadFile(handle);
        {
            // protobuff searilaze as/from string
            // seems like direct parse to/from stream is possible
            std::fstream input(filename, std::ios::in | std::ios::binary);
            m_Grid2D->ParseFromIstream(&input);
        }
        m_streamProvider->endReadFile(handle);
    }
    return m_Grid2D;
}


int Grid2DEntitydata::setData(float x1, float y1, float z1,
                                   float x2, float y2, float z2, mapit::entitytypes::Grid2DType &data,
                                   int lod)

{
    int result = -1;
    mapit::ReadWriteHandle handle;
    std::string filename = m_streamProvider->startWriteFile(handle);
    {
        m_Grid2D = data;
        std::fstream output(filename, std::ios::out | std::ios::trunc | std::ios::binary);
        result = m_Grid2D->SerializeToOstream(&output);
    }
    m_streamProvider->endWriteFile(handle);
    return result;
}

mapit::entitytypes::Grid2DType Grid2DEntitydata::getData(int lod)
{
    return getData(-std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                   false, lod);
}

int Grid2DEntitydata::setData(mapit::entitytypes::Grid2DType &data, int lod)
{
    return setData(-std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                   data, lod);
}

void Grid2DEntitydata::gridCellAt(float   x, float   y, float   z,
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

int Grid2DEntitydata::getEntityBoundingBox(float &x1, float &y1, float &z1,
                                              float &x2, float &y2, float &z2)
{

 //   pcl::PointXYZ min;
 //   pcl::PointXYZ max;
 //   pcl::PointCloud<pcl::PointXYZ> pxyz;
 //   pcl::fromPCLPointCloud2<pcl::PointXYZ>(*m_pointcloud, pxyz);
 //   pcl::getMinMax3D(pxyz, min, max);
 //   x1 = min.x;
 //   y1 = min.y;
 //   z1 = min.z;
 //   x2 = max.x;
 //   y2 = max.y;
 //   z2 = max.z;

    // TODO Do something usefull here
    x1 = -std::numeric_limits<float>::infinity();
    y1 = -std::numeric_limits<float>::infinity();
    z1 = -std::numeric_limits<float>::infinity();
    x2 = +std::numeric_limits<float>::infinity();
    y2 = +std::numeric_limits<float>::infinity();
    z2 = +std::numeric_limits<float>::infinity();

    return 0;
}

mapit::istream *Grid2DEntitydata::startReadBytes(mapit::uint64_t start, mapit::uint64_t len)
{
    return m_streamProvider->startRead(start, len);
}

void Grid2DEntitydata::endRead(mapit::istream *&strm)
{
    m_streamProvider->endRead(strm);
}

mapit::ostream *Grid2DEntitydata::startWriteBytes(mapit::uint64_t start, mapit::uint64_t len)
{
    return m_streamProvider->startWrite(start, len);
}

void Grid2DEntitydata::endWrite(mapit::ostream *&strm)
{
    m_streamProvider->endWrite(strm);
}

size_t Grid2DEntitydata::size() const
{
    m_streamProvider->getStreamSize();
}

// Win32 does not like anything but void pointers handled between libraries
// For Unix there would be a hack to use a "custom deleter" which is given to the library to clean up the created memory
// the common denominator is to build pointer with custom deleter in our main programm and just exchange void pointers and call delete when we are done
//std::shared_ptr<mapit::AbstractEntitydata> createEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
//void* createEntitydata(std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider)
void deleteEntitydataPcd(mapit::AbstractEntitydata *ld)
{
    Grid2DEntitydata *p = dynamic_cast<Grid2DEntitydata*>(ld);
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
    //return std::shared_ptr<AbstractEntitydata>(new Grid2DEntitydata( streamProvider ), deleteWrappedLayerData);
    *out = std::shared_ptr<mapit::AbstractEntitydata>(new Grid2DEntitydata( streamProvider ), deleteEntitydataPcd);
}

