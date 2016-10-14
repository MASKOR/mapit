#include "module.h"
#include "libs/layertypes_collection/pointcloud2/include/pointcloudlayer.h"
#include "modules/versioning/checkoutraw.h"
#include "modules/operationenvironment.h"
#include "modules/versioning/checkoutraw.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <memory>
#include "upns_errorcodes.h"
#include "param.pb.h"
#include <QJsonDocument>
#include <QJsonObject>

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
//    LoadPointcloudParams params;
//    params.ParseFromString( env->getParameters() );
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());
    upnsPointcloud2Ptr pc2( new pcl::PCLPointCloud2);

    pcl::PCDReader reader;
    std::string filename = params["filename"].toString().toStdString();
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

    std::string target = params["target"].toString().toStdString();

    upnsSharedPointer<Entity> pclEntity(new Entity);
    pclEntity->set_type(POINTCLOUD2);
    StatusCode s = env->getCheckout()->storeEntity(target, pclEntity);
    if(!upnsIsOk(s))
    {
        log_error("Failed to create entity.");
    }
    upnsSharedPointer<AbstractEntityData> abstractEntityData = env->getCheckout()->getEntityDataForReadWrite( target );

    upnsSharedPointer<PointcloudEntitydata> entityData = upns::static_pointer_cast<PointcloudEntitydata>(abstractEntityData);
    entityData->setData( pc2 );

    OperationDescription out;
    out.set_operatorname(OPERATOR_NAME);
    out.set_operatorversion(OPERATOR_VERSION);
//    OperationParameter *outTarget = out.add_params();
//    outTarget->set_key("target");
////    outTarget->set_mapval( map->id() );
////    outTarget->set_layerval( layer->id() );
////    outTarget->set_entityval( entity->id() );
//    OperationParameter *outMapname = out.add_params();
//    outMapname->set_key("mapname");
//    outMapname->set_strval( map->name() );
    env->setOutputDescription( out.SerializeAsString() );
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Pcd File", "fhac", OPERATOR_VERSION, upns::LayerType::POINTCLOUD2, &operate_load_pointcloud)
