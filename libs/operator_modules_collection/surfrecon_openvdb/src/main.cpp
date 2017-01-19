#include "module.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include "libs/layertypes_collection/pointcloud2/include/pointcloudlayer.h"
#include "libs/layertypes_collection/openvdb/include/openvdblayer.h"
#include <openvdb/Grid.h>
#include <openvdb/tools/ParticlesToLevelSet.h>
#include <openvdb/tools/LevelSetUtil.h>
#include "modules/versioning/checkoutraw.h"
#include "modules/operationenvironment.h"
#include <iostream>
#include <memory>
#include "upns_errorcodes.h"
#include "modules/versioning/checkoutraw.h"
#include "json11.hpp"

template<typename ParticlePointT>
class ParticleFromCloudList {
    typedef typename pcl::PointCloud<ParticlePointT>::Ptr ParticlePointCloudPtr;

    ParticlePointCloudPtr m_cloud;

public:

    typedef openvdb::Vec3R  PosType;
    typedef openvdb::Vec3R  value_type;

    ParticleFromCloudList(ParticlePointCloudPtr cloud)
        :m_cloud( cloud)
    {}

    // Return the total number of particles in list.
    // Always required!
    size_t size() const { return m_cloud->points.size(); }

    // Get the world space position of n'th particle.
    // Required by ParticledToLevelSet::rasterizeSphere(*this,radius).
    void getPos(size_t n, openvdb::Vec3R& xyz) const
    {
        xyz.x() = m_cloud->points[n].x;
        xyz.y() = m_cloud->points[n].y;
        xyz.z() = m_cloud->points[n].z;
    }

//    void getNormal(size_t n, openvdb::Vec3R& xyz) const
//    {
//        xyz.x() = m_cloud->points[n].normal[0];
//        xyz.y() = m_cloud->points[n].normal[0];
//        xyz.z() = m_cloud->points[n].normal[0];
//    }
};

// JSON:
// - radius: size of spheres to create
// - voxelsize: size of voxels in grid. Resolution of volume
// - input: input pointcloud
// - output: output openvdb
// - target: input and output at the same time
upns::StatusCode operate_tolevelset(upns::OperationEnvironment* env)
{
    std::string jsonErr;
    json11::Json params = json11::Json::parse(env->getParameters(), jsonErr);
    if ( ! jsonErr.empty() ) {
        // can't parth json
        // TODO: good error msg
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    double spheresize = params["radius"].number_value();

    if(spheresize == 0.0)
    {
        spheresize = 0.01f;
    }

    double voxelsize = params["voxelsize"].number_value();

    if(voxelsize == 0.0)
    {
        voxelsize = 0.01f;
    }
    if(voxelsize >= spheresize*2.0)
    {
        log_warn("radius of spheres is too small for given voxel resolution. Please try a higher \"radius\" or smaller \"voxelsize\".");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    std::string input =  params["input"].string_value();
    std::string output = params["output"].string_value();
    if(input.empty())
    {
        input = params["target"].string_value();
        if(input.empty())
        {
            log_error("no input specified");
            return UPNS_STATUS_INVALID_ARGUMENT;
        }
    }
    if(output.empty())
    {
        output = params["target"].string_value();
        if(output.empty())
        {
            log_error("no output specified");
            return UPNS_STATUS_INVALID_ARGUMENT;
        }
    }

    upnsSharedPointer<AbstractEntitydata> abstractEntitydataInput = env->getCheckout()->getEntitydataReadOnly( input );
    if(!abstractEntitydataInput)
    {
        log_error("input does not exist ore is not readable.");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    upnsSharedPointer<PointcloudEntitydata> entityDataInput = upns::static_pointer_cast<PointcloudEntitydata>( abstractEntitydataInput );
    upnsPointcloud2Ptr inputPcd = entityDataInput->getData();

    upnsFloatGridPtr outputFloatGrid;
    upnsSharedPointer<Entity> ent = env->getCheckout()->getEntity(output);
    if(ent)
    {
        log_info("Output grid already exists. ignoring voxelsize.");
        upnsSharedPointer<AbstractEntitydata> abstractEntitydataOutput = env->getCheckout()->getEntitydataReadOnly( output );
        if(!abstractEntitydataOutput)
        {
            log_error("could not read output grid");
            return UPNS_STATUS_INVALID_ARGUMENT;
        }
        upnsSharedPointer<FloatGridEntitydata> entityDataOutput = upns::static_pointer_cast<FloatGridEntitydata>( abstractEntitydataOutput );
        if(!entityDataOutput)
        {
            log_error("could not cast output to FloatGrid");
            return UPNS_STATUS_INVALID_ARGUMENT;
        }
        outputFloatGrid = entityDataOutput->getData();
    }
    else
    {
        ent = upnsSharedPointer<Entity>(new Entity);
        ent->set_type(OPENVDB);
        StatusCode s = env->getCheckout()->storeEntity(output, ent);
        if(!upnsIsOk(s))
        {
            log_error("Failed to create entity.");
            return UPNS_STATUS_ERR_DB_IO_ERROR;
        }
        outputFloatGrid = openvdb::createLevelSet<openvdb::FloatGrid>( voxelsize );
    }

    /// Input validation finished

    openvdb::tools::ParticlesToLevelSet<openvdb::FloatGrid> particlesToLevelset( *outputFloatGrid );

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*inputPcd, *pc );
    ParticleFromCloudList<pcl::PointXYZ> spheres( pc );
    log_info("Voxelize " + std::to_string(spheres.size()) + " Points as Spheres.");
    particlesToLevelset.rasterizeSpheres( spheres, spheresize );

    upnsSharedPointer<AbstractEntitydata> abstractEntitydataOutput = env->getCheckout()->getEntitydataForReadWrite( output );
    if(!abstractEntitydataOutput)
    {
        log_error("could not read output grid");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    upnsSharedPointer<FloatGridEntitydata> entityDataOutput = upns::static_pointer_cast<FloatGridEntitydata>( abstractEntitydataOutput );
    if(!entityDataOutput)
    {
        log_error("could not cast output to FloatGrid");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    entityDataOutput->setData(outputFloatGrid);

    OperationDescription out;
    out.set_operatorname(OPERATOR_NAME);
    out.set_operatorversion(OPERATOR_VERSION);
    env->setOutputDescription( out.SerializeAsString() );
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "make a levelset out of a pointcloud using pcl and openvdb", "fhac", OPERATOR_VERSION, upns::LayerType::POINTCLOUD, &operate_tolevelset)
