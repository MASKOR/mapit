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

#ifndef GRID2DLAYER_H
#define GRID2DLAYER_H

#include <mapit/entitydata.h>
#include <mapit/operators/serialization/abstractentitydataprovider.h>
#include <mapit/msgs/datastructs.pb.h>
#include <pcl/point_types.h>

#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif

// Not a good idea because voxelgridfilter uses pcl smart pointers (boost)
namespace mapit
{
namespace entitytypes
{
typedef std::shared_ptr<mapit::msgs::Grid2D> Grid2DType;
}
}
extern "C"
{
//Note: Not possible in MSVC/Windows. Breaks shared pointers exchange
//MODULE_EXPORT std::shared_ptr<mapit::AbstractEntitydata> createEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider);

MODULE_EXPORT void createEntitydata(std::shared_ptr<mapit::AbstractEntitydata> *out, std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider);

}

//#include <boost/preprocessor/repetition.hpp>

//#define FOR_COMMON_POINTTYPES(fn, pointcloudEntitydata) \
//    switch(pointcloudEntitydata.)



class Grid2DEntitydata : public mapit::Entitydata<mapit::msgs::Grid2D>
{
public:
    static const char* TYPENAME();
    //
    /**
     * @brief The Representation enum of some Representations witch most algorithms will be able to use.
     * Note that all representations of pcl is possible and all fields of pcd can be used.
     * However, implementation of all types can be timeconsuming. At least these types should be supported by most
     * operations.
     */
    /*
    enum Representation {
        Rep_XY, // = pcl::PointXY,
        Rep_XYZ, // = pcl::PointXYZ,
        Rep_XYZI, // = pcl::PointXYZI,
        Rep_XYZRGB, // = pcl::PointXYZRGB,
        Rep_XYZRGBA, // = pcl::PointXYZRGBA,
        //Rep_XYZNormal, // = pcl::PointXYZ,
        Rep_XYZINormal, // = pcl::PointXYZINormal,
        Rep_XYZRGBNormal, // = pcl::PointXYZRGBNormal,
        //Rep_XYZRGBANormal, // = pcl::PointXYZRGB,
        Rep_Other
    };*/

    Grid2DEntitydata(std::shared_ptr<mapit::AbstractEntitydataProvider> streamProvider);

    const char*         type() const;
    bool                hasFixedGrid() const;
    bool                canSaveRegions() const;
    mapit::entitytypes::Grid2DType getData(float x1, float y1, float z1,float x2, float y2, float z2, bool clipMode, int lod);
    int                 setData(float x1, float y1, float z1, float x2, float y2, float z2,  mapit::entitytypes::Grid2DType &data, int lod);

   mapit::entitytypes::Grid2DType  getData(int lod = 0);
    int                 setData(mapit::entitytypes::Grid2DType &data, int lod = 0);

    void gridCellAt(float   x, float   y, float   z,
                    float &x1, float &y1, float &z1,
                    float &x2, float &y2, float &z2) const;

    int getEntityBoundingBox(float &x1, float &y1, float &z1,
                             float &x2, float &y2, float &z2);

    mapit::istream *startReadBytes(mapit::uint64_t start, mapit::uint64_t len);
    void endRead(mapit::istream *&strm);

    mapit::ostream *startWriteBytes(mapit::uint64_t start, mapit::uint64_t len);
    void endWrite(mapit::ostream *&strm);

    size_t size() const;
private:
    std::shared_ptr<mapit::AbstractEntitydataProvider> m_streamProvider;
    //pcl::PointCloud<pcl::PointXYZ> m_pointcloud;
    mapit::entitytypes::Grid2DType m_Grid2D;
};

#endif
