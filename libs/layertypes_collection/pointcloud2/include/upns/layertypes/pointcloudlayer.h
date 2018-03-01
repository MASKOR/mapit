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

#ifndef POINTCLOUDLAYER_H
#define POINTCLOUDLAYER_H

#include <upns/entitydata.h>
#include <upns/operators/serialization/abstractentitydataprovider.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>

using namespace upns;

// TODO: Is import case needed? (dynamic linking?)
#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif

// Not a good idea because voxelgridfilter uses pcl smart pointers (boost)
typedef std::shared_ptr<pcl::PCLPointCloud2> upnsPointcloud2Ptr;


extern "C"
{
//Note: Not possible in MSVC/Windows. Breaks shared pointers exchange
//MODULE_EXPORT std::shared_ptr<AbstractEntitydata> createEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider);

MODULE_EXPORT void createEntitydata(std::shared_ptr<AbstractEntitydata> *out, std::shared_ptr<AbstractEntitydataProvider> streamProvider);

}


//#include <boost/preprocessor/repetition.hpp>

//#define FOR_COMMON_POINTTYPES(fn, pointcloudEntitydata) \
//    switch(pointcloudEntitydata.)



class PointcloudEntitydata : public Entitydata<pcl::PCLPointCloud2>
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
    };

    PointcloudEntitydata(std::shared_ptr<AbstractEntitydataProvider> streamProvider);

    const char*         type() const;
    bool                hasFixedGrid() const;
    bool                canSaveRegions() const;
    upnsPointcloud2Ptr  getData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                bool clipMode,
                                int lod = 0);
    int                 setData(upnsReal x1, upnsReal y1, upnsReal z1,
                                upnsReal x2, upnsReal y2, upnsReal z2,
                                upnsPointcloud2Ptr &data,
                                int lod = 0);

    upnsPointcloud2Ptr  getData(int lod = 0);
    int                 setData(upnsPointcloud2Ptr &data, int lod = 0);

    void gridCellAt(upnsReal   x, upnsReal   y, upnsReal   z,
                    upnsReal &x1, upnsReal &y1, upnsReal &z1,
                    upnsReal &x2, upnsReal &y2, upnsReal &z2) const;

    int getEntityBoundingBox(upnsReal &x1, upnsReal &y1, upnsReal &z1,
                             upnsReal &x2, upnsReal &y2, upnsReal &z2);

    upnsIStream *startReadBytes(upnsuint64 start, upnsuint64 len);
    void endRead(upnsIStream *&strm);

    upnsOStream *startWriteBytes(upnsuint64 start, upnsuint64 len);
    void endWrite(upnsOStream *&strm);

    size_t size() const;
private:
    std::shared_ptr<AbstractEntitydataProvider> m_streamProvider;
    //pcl::PointCloud<pcl::PointXYZ> m_pointcloud;
    upnsPointcloud2Ptr m_pointcloud;
};

#endif
