#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <memory>
#include <upns/errorcodes.h>
#include <upns/operators/versioning/checkoutraw.h>
#include "json11.hpp"

upns::StatusCode operate_vxg(upns::OperationEnvironment* env)
{
    std::string jsonErr;
    json11::Json params = json11::Json::parse(env->getParameters(), jsonErr);
    if ( ! jsonErr.empty() ) {
        // can't parth json
        // TODO: good error msg
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    double leafSize = params["leafsize"].number_value();

    if(leafSize == 0.0)
    {
        leafSize = 0.01f;
    }

    std::string target = params["target"].string_value();

    upnsSharedPointer<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    upnsSharedPointer<PointcloudEntitydata> entityData = upns::static_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
    upnsPointcloud2Ptr pc2 = entityData->getData();

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    pcl::PCLPointCloud2ConstPtr stdPc2( pc2.get(), [](pcl::PCLPointCloud2*){});
    sor.setInputCloud(stdPc2);
    sor.setLeafSize (leafSize, leafSize, leafSize);

    upnsPointcloud2Ptr cloud_filtered(new pcl::PCLPointCloud2 ());
    sor.filter (*cloud_filtered);
    std::stringstream strstr;
    strstr << "new pointcloudsize " << cloud_filtered->width;
    log_info( strstr.str() );

    entityData->setData(cloud_filtered);

    OperationDescription out;
    out.set_operatorname(OPERATOR_NAME);
    out.set_operatorversion(OPERATOR_VERSION);
//    OperationParameter *outTarget = out.add_params();
//    outTarget->set_key("target");
////    outTarget->set_mapval( map->id() );
////    outTarget->set_layerval( layer->id() );
////    outTarget->set_entityval( entity->id() );
//    OperationParameter *outMapname = out.add_params();
//    outMapname->set_key("leafsize");
//    outMapname->set_realval( leafSize );
    env->setOutputDescription( out.SerializeAsString() );
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "use pcl voxelgrid filter on a pointcloud", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, &operate_vxg)
