#ifndef POINTCLOUDLAYER_H
#define POINTCLOUDLAYER_H

#include "entitydata.h"
#include "modules/serialization/abstractentitydatastreamprovider.h"
#include <pcl/PCLPointCloud2.h>

using namespace upns;

// TODO: Is import case needed? (dynmic linking?)
#ifdef _WIN32
#define MODULE_EXPORT __declspec(dllexport)
#else
#define MODULE_EXPORT // empty
#endif

// Not a good idea because voxelgridfilter uses pcl smart pointers (bbost)
typedef upnsSharedPointer<pcl::PCLPointCloud2> upnsPointcloud2Ptr;
//TODO: will not compile... guru meditation
//using upnsPointcloud2Ptr = pcl::PCLPointCloud2Ptr;

extern "C"
{
//Note: Not possible in MSVC/Windows. Breaks shared pointers exchange
//MODULE_EXPORT upnsSharedPointer<AbstractEntityData> createEntitydata(upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider);
MODULE_EXPORT void createEntitydata(upnsSharedPointer<AbstractEntityData> *out, upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider);
//MODULE_EXPORT void deleteEntitydata(upnsSharedPointer<AbstractEntityData> streamProvider);
}

class PointcloudEntitydata : public EntityData<pcl::PCLPointCloud2>
{
public:
    //
    /**
     * @brief The Representation enum of some Representations witch most algorithms will be able to use.
     * Note that all representations of pcl is possible and all fields of pcd can be used.
     * However, implementation of all types can be timeconsuming. At least these types should be supported by most
     * operations.
     */
    enum Representation {
        Rep_XYZ,
        Rep_XYZI,
        Rep_XYZRGB,
        Rep_XYZRGBA,
        Rep_XYZNormal,
        Rep_XYZINormal,
        Rep_XYZRGBNormal,
        Rep_XYZRGBANormal,
        Rep_Other
    };

    PointcloudEntitydata(upnsSharedPointer<AbstractEntityDataStreamProvider> streamProvider);

    LayerType           layerType() const;
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

private:
    upnsSharedPointer<AbstractEntityDataStreamProvider> m_streamProvider;
    //pcl::PointCloud<pcl::PointXYZ> m_pointcloud;
    upnsPointcloud2Ptr m_pointcloud;

};

#endif
