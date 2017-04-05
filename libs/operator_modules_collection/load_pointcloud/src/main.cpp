#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <memory>
#include <upns/errorcodes.h>
//#include "param.pb.h"
#include "json11.hpp"

using namespace mapit::msgs;

bool field_present(const std::string& name, const  std::vector<pcl::PCLPointField>& flist){
    for(int i=0; i<flist.size(); i++){
        if (flist[i].name == name) return true;
    }
    return false;
}

void chooseDefaultRepresentation ( const std::vector<pcl::PCLPointField>& flist )
{
    if ( field_present("rgba", flist)){
        //return new PointCloudRGBFactory<pcl::PointXYZRGBA, pcl::RGB>();
    } else if ( field_present("rgb", flist)){
            //return new PointCloudRGBFactory<pcl::PointXYZRGB, pcl::RGB>();
    } else if ( field_present("intensity", flist)){
        //return NULL; //PointCloudIFactory<pcl::PointXYZI, pcl::Intensity>;
    } else if ( field_present("label", flist)){
        //return new PointCloudLabelFactory<pcl::PointXYZ, pcl::Label>();
    }
    //return new PointCloudCRangeFactory<>("z");
}

upns::StatusCode operate_load_pointcloud(upns::OperationEnvironment* env)
{
    upnsPointcloud2Ptr pc2( new pcl::PCLPointCloud2);

    std::string jsonErr;
    json11::Json params = json11::Json::parse(env->getParameters(), jsonErr);

    if ( ! jsonErr.empty() ) {
        // can't parse json
        // TODO: good error msg
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    pcl::PCDReader reader;
    std::string filename = params["filename"].string_value();
    if(filename.empty())
    {
        log_error("parameter \"filename\" missing");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    if ( reader.read(filename, *pc2) < 0 )
    {
        log_error("Couldn't read file" + filename);
        return UPNS_STATUS_FILE_NOT_FOUND;
    }

    std::string target = params["target"].string_value();

    std::shared_ptr<Entity> pclEntity(new Entity);
    pclEntity->set_type(PointcloudEntitydata::TYPENAME());
    StatusCode s = env->getCheckout()->storeEntity(target, pclEntity);
    if(!upnsIsOk(s))
    {
        log_error("Failed to create entity.");
    }
    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );

    std::shared_ptr<PointcloudEntitydata> entityData = std::static_pointer_cast<PointcloudEntitydata>(abstractEntitydata);
    entityData->setData( pc2 );

//    OperationDescription out;
//    out.set_operatorname(OPERATOR_NAME);
//    out.set_operatorversion(OPERATOR_VERSION);
//    OperationParameter *outTarget = out.add_params();
//    outTarget->set_key("target");
////    outTarget->set_mapval( map->id() );
////    outTarget->set_layerval( layer->id() );
////    outTarget->set_entityval( entity->id() );
//    OperationParameter *outMapname = out.add_params();
//    outMapname->set_key("mapname");
//    outMapname->set_strval( map->name() );
//    env->setOutputDescription( out.SerializeAsString() );
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Pcd File", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, &operate_load_pointcloud)
