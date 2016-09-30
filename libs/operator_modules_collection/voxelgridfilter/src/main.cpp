#include "module.h"
#include "libs/layertypes_collection/pointcloud2/include/pointcloudlayer.h"
#include "modules/versioning/checkoutraw.h"
#include "operationenvironment.h"
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <memory>
#include "error.h"
#include "modules/versioning/checkoutraw.h"
#include <QJsonDocument>
#include <QJsonObject>

upns::StatusCode operate_vxg(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    double leafSize = params["leafsize"].toDouble();
    if(leafSize == 0.0)
    {
        leafSize = 0.01f;
    }

    std::string target = params["target"].toString().toStdString();

    upnsSharedPointer<AbstractEntityData> abstractEntityData = env->getCheckout()->getEntityDataForReadWrite( target );
    upnsSharedPointer<PointcloudEntitydata> entityData = upns::static_pointer_cast<PointcloudEntitydata>( abstractEntityData );
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

UPNS_MODULE(OPERATOR_NAME, "use pcl voxelgrid filter on a pointcloud", "fhac", OPERATOR_VERSION, upns::LayerType::POINTCLOUD2, &operate_vxg)
