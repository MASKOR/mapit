#include <upns/operators/module.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <iostream>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <memory>
#include <upns/errorcodes.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <QJsonDocument>
#include <QJsonObject>

using namespace mapit::msgs;

upns::StatusCode operate_ctr(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();

    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    std::shared_ptr<PointcloudEntitydata> entityData = std::static_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
    upnsPointcloud2Ptr pc2 = entityData->getData();
    pcl::PointCloud<pcl::PointXYZ> pc; // TODO: make generic
    pcl::fromPCLPointCloud2(*pc2, pc);

    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::getMinMax3D(pc, min, max);
    Eigen::Vector4f ctr((min.x+max.x)/2.0f, (min.y+max.y)/2.0f, (min.z+max.z)/2.0f, 1.0f);
    pcl::PointCloud<pcl::PointXYZ> pcCtr; // TODO: make generic
    pcl::demeanPointCloud(pc, ctr, pcCtr);

    upnsPointcloud2Ptr pc2Out(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2( pc, *pc2Out);

    entityData->setData(pc2Out);
//    OperationDescription out;
//    out.set_operatorname(OPERATOR_NAME);
//    out.set_operatorversion(OPERATOR_VERSION);
//    OperationParameter *outTarget = out.add_params();
//    outTarget->set_key("target");
////    outTarget->set_mapval( map->id() );
////    outTarget->set_layerval( layer->id() );
////    outTarget->set_entityval( entity->id() );
//    OperationParameter *outMapname = out.add_params();
//    outMapname->set_key("leafsize");
//    outMapname->set_realval( leafSize );
//    env->setOutputDescription( out.SerializeAsString() );
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "use point of mass as origin of the pointcloud", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, &operate_ctr)
