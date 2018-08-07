/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include "mapit/layertypes/octomaplayer.h"
#include <mapit/logging.h>
#include <sstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h> // for bb

char const *
mapit::entitytypes::Octomap::TYPENAME()
{
    return PROJECT_NAME;
}

mapit::entitytypes::Octomap::Octomap(std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider) :
    m_streamProvider( streamProvider )
  , m_octomap( NULL )
{
}

char const *
mapit::entitytypes::Octomap::type() const
{
    return mapit::entitytypes::Octomap::TYPENAME();
}

bool
mapit::entitytypes::Octomap::hasFixedGrid() const
{
    return false;
}

bool
mapit::entitytypes::Octomap::canSaveRegions() const
{
    return false;
}

std::shared_ptr<octomap::OcTree>
mapit::entitytypes::Octomap::getData(  float x1, float y1, float z1
                                     , float x2, float y2, float z2
                                     , bool clipMode, int lod)
{
    if(m_octomap == NULL)
    {
        mapit::ReadWriteHandle handle;
        std::string filename = m_streamProvider->startReadFile(handle);
        {
            m_octomap = std::make_shared<octomap::OcTree>( filename );
        }
        m_streamProvider->endReadFile(handle);
    }
    return m_octomap;
}

std::shared_ptr<octomap::OcTree>
mapit::entitytypes::Octomap::getData(int lod)
{
    return getData(  -std::numeric_limits<float>::infinity()
                   , -std::numeric_limits<float>::infinity()
                   , -std::numeric_limits<float>::infinity()
                   ,  std::numeric_limits<float>::infinity()
                   ,  std::numeric_limits<float>::infinity()
                   ,  std::numeric_limits<float>::infinity()
                   , false, lod);
}


int
mapit::entitytypes::Octomap::setData(  float x1, float y1, float z1
                                     , float x2, float y2, float z2
                                     , std::shared_ptr<octomap::OcTree> &data, int lod)
{
    mapit::ReadWriteHandle handle;
    std::string filename = m_streamProvider->startWriteFile(handle);
    {
        m_octomap = data;
        m_octomap->writeBinary( filename );
    }
    m_streamProvider->endWriteFile(handle);
    return -1;
}

int
mapit::entitytypes::Octomap::setData(std::shared_ptr<octomap::OcTree> &data, int lod)
{
    return setData(  -std::numeric_limits<float>::infinity()
                   , -std::numeric_limits<float>::infinity()
                   , -std::numeric_limits<float>::infinity()
                   ,  std::numeric_limits<float>::infinity()
                   ,  std::numeric_limits<float>::infinity()
                   ,  std::numeric_limits<float>::infinity()
                   , data, lod);
}

void
mapit::entitytypes::Octomap::gridCellAt(  float   x, float   y, float   z
                                        , float &x1, float &y1, float &z1
                                        , float &x2, float &y2, float &z2) const
{
    x1 = -std::numeric_limits<float>::infinity();
    y1 = -std::numeric_limits<float>::infinity();
    z1 = -std::numeric_limits<float>::infinity();
    x2 = +std::numeric_limits<float>::infinity();
    y2 = +std::numeric_limits<float>::infinity();
    z2 = +std::numeric_limits<float>::infinity();
}

int
mapit::entitytypes::Octomap::getEntityBoundingBox(  float &x1, float &y1, float &z1
                                                  , float &x2, float &y2, float &z2)
{
    // make sure data is loaded if possible
    getData();

    if ( ! m_octomap ) {
        // when data dosn't exists
        x1 = -0;
        y1 = -0;
        z1 = -0;
        x2 =  0;
        y2 =  0;
        z2 =  0;
    } else {
        // when data is readable use values from octomap
        double x, y, z = 0;
        m_octomap->getMetricMax(x, y, z);
        x1 = -static_cast<float>(x);
        y1 = -static_cast<float>(y);
        z1 = -static_cast<float>(z);
        x2 =  static_cast<float>(x);
        y2 =  static_cast<float>(y);
        z2 =  static_cast<float>(z);
    }

    return 0;
}

mapit::istream *
mapit::entitytypes::Octomap::startReadBytes(mapit::uint64_t start, mapit::uint64_t len)
{
    return m_streamProvider->startRead(start, len);
}

void
mapit::entitytypes::Octomap::endRead(mapit::istream *&strm)
{
    m_streamProvider->endRead(strm);
}

mapit::ostream *
mapit::entitytypes::Octomap::startWriteBytes(mapit::uint64_t start, mapit::uint64_t len)
{
    return m_streamProvider->startWrite(start, len);
}

void
mapit::entitytypes::Octomap::endWrite(mapit::ostream *&strm)
{
    m_streamProvider->endWrite(strm);
}

size_t
mapit::entitytypes::Octomap::size() const
{
    m_streamProvider->getStreamSize();
}

// Win32 does not like anything but void pointers handled between libraries
// For Unix there would be a hack to use a "custom deleter" which is given to the library to clean up the created memory
// the common denominator is to build pointer with custom deleter in our main programm and just exchange void pointers and call delete when we are done
//std::shared_ptr<AbstractEntitydata> createEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
//void* createEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider)
//TODO: BIG TODO: Make libraries have a deleteEntitydata function and do not use shared pointers between libraries.
// TfEntitydata was deleted here although it was a plymesh
void deleteEntitydataOctomap(mapit::AbstractEntitydata *ld)
{
    mapit::entitytypes::Octomap *p = dynamic_cast<mapit::entitytypes::Octomap*>(ld);
    if(p)
    {
        delete p;
    }
    else
    {
        log_error("Wrong entitytype");
    }
}

void
createEntitydata(std::shared_ptr<mapit::AbstractEntitydata> *out, std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider)
{
    //return std::shared_ptr<AbstractEntitydata>(new mapit::entitytypes::Octomap( streamProvider ), deleteWrappedLayerData);
    *out = std::shared_ptr<mapit::AbstractEntitydata>(new mapit::entitytypes::Octomap( streamProvider ), deleteEntitydataOctomap);
}

