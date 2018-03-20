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

#include <mapit/operators/module.h>
#include <mapit/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <mapit/layertypes/pointcloudlayer.h>
#include <mapit/layertypes/openvdblayer.h>
#include <openvdb/Grid.h>
#include <openvdb/tools/ParticlesToLevelSet.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>
#include <iostream>
#include <memory>
#include <mapit/errorcodes.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

using namespace mapit::msgs;

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
mapit::StatusCode operate_tolevelset(mapit::OperationEnvironment* env)
{
    const char* szParams = env->getParameters().c_str();
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(szParams, env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string input =  params["input"].toString().toStdString();
    std::string output = params["output"].toString().toStdString();
    if(input.empty())
    {
        input = params["target"].toString().toStdString();
        if(input.empty())
        {
            log_error("no input specified");
            return MAPIT_STATUS_INVALID_ARGUMENT;
        }
    }
    if(output.empty())
    {
        output = params["target"].toString().toStdString();
        if(output.empty())
        {
            log_error("no output specified");
            return MAPIT_STATUS_INVALID_ARGUMENT;
        }
    }

    double spheresize = params["radius"].toDouble();
    if(spheresize == 0.0)
    {
        spheresize = 0.01f;
    }
    double voxelsize = params["voxelsize"].toDouble();
    if(voxelsize == 0.0)
    {
        voxelsize = 0.04f;
    }
    if(voxelsize >= spheresize*2.1)
    {
        log_warn("radius of spheres is too small for given voxel resolution. Please try a higher \"radius\" or smaller \"voxelsize\".");
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }

    std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydataInput = env->getWorkspace()->getEntitydataReadOnly( input );
    if(!abstractEntitydataInput)
    {
        log_error("input does not exist or is not readable.");
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }
    std::shared_ptr<PointcloudEntitydata> entityDataInput = std::dynamic_pointer_cast<PointcloudEntitydata>( abstractEntitydataInput );
    if(entityDataInput == nullptr)
    {
        log_error("Wrong type");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    mapit::entitytypes::Pointcloud2Ptr inputPcd = entityDataInput->getData();

    FloatGridPtr outputFloatGrid;
    std::shared_ptr<Entity> ent = env->getWorkspace()->getEntity(output);
    if(ent && false)
    {
        //TODO: at the moment always a new grid should be created
        log_info("Output grid already exists. ignoring voxelsize.");
        std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydataOutput = env->getWorkspace()->getEntitydataReadOnly( output );
        if(!abstractEntitydataOutput)
        {
            log_error("could not read output grid");
            return MAPIT_STATUS_INVALID_ARGUMENT;
        }
        std::shared_ptr<FloatGridEntitydata> entityDataOutput = std::dynamic_pointer_cast<FloatGridEntitydata>( abstractEntitydataOutput );
        if(!entityDataOutput)
        {
            log_error("could not cast output to FloatGrid");
            return MAPIT_STATUS_INVALID_ARGUMENT;
        }
        outputFloatGrid = entityDataOutput->getData();
    }
    else
    {
        ent = std::shared_ptr<Entity>(new Entity);
        ent->set_type(FloatGridEntitydata::TYPENAME());
        mapit::StatusCode s = env->getWorkspace()->storeEntity(output, ent);
        if(!mapitIsOk(s))
        {
            log_error("Failed to create entity.");
            return MAPIT_STATUS_ERR_DB_IO_ERROR;
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

    std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydataOutput = env->getWorkspace()->getEntitydataForReadWrite( output );
    if(!abstractEntitydataOutput)
    {
        log_error("could not read output grid");
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }
    std::shared_ptr<FloatGridEntitydata> entityDataOutput = std::dynamic_pointer_cast<FloatGridEntitydata>( abstractEntitydataOutput );
    if(!entityDataOutput)
    {
        log_error("could not cast output to FloatGrid");
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }
    entityDataOutput->setData(outputFloatGrid);

//    OperationDescription out;
//    out.set_operatorname(OPERATOR_NAME);
//    out.set_operatorversion(OPERATOR_VERSION);
//    env->setOutputDescription( out.SerializeAsString() );
    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME, "make a levelset out of a pointcloud using pcl and openvdb", "fhac", OPERATOR_VERSION, FloatGridEntitydata_TYPENAME, &operate_tolevelset)
