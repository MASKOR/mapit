#include <upns/logging.h>
#include <upns/operators/module.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <upns/layertypes/tflayer.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <iostream>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <memory>
#include <upns/errorcodes.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>

using namespace mapit::msgs;

upns::StatusCode operate_ctr(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();

    bool useAxisAlignedBoundingBox = params["useAABB"].toBool();

    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
    if(entityData == nullptr)
    {
        log_error("Wrong type");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    upnsPointcloud2Ptr pc2 = entityData->getData();
    pcl::PointCloud<pcl::PointXYZ> pc; // TODO: make generic
    pcl::fromPCLPointCloud2(*pc2, pc);

    Eigen::Vector4f ctr;
    if(useAxisAlignedBoundingBox)
    {
        pcl::PointXYZ min;
        pcl::PointXYZ max;
        pcl::getMinMax3D(pc, min, max);
        log_info("Centroid to origin: Minimum was ("
                 + std::to_string(min.x) + ", "
                 + std::to_string(min.y) + ", "
                 + std::to_string(min.z) + "), Maximum: ("
                 + std::to_string(max.x) + ", "
                 + std::to_string(max.y) + ", "
                 + std::to_string(max.z) + ")");
        ctr = Eigen::Vector4f((min.x+max.x)/2.0f, (min.y+max.y)/2.0f, (min.z+max.z)/2.0f, 1.0f);
    }
    else
    {
        pcl::compute3DCentroid(pc, ctr);
        log_info("Centroid to origin: Centroid was ("
                 + std::to_string(ctr[0]) + ", "
                 + std::to_string(ctr[1]) + ", "
                 + std::to_string(ctr[2]) + ")");
    }

    pcl::PointCloud<pcl::PointXYZ> pcCtr; // TODO: make generic
    pcl::demeanPointCloud(pc, ctr, pcCtr);

    upnsPointcloud2Ptr pc2Out(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2( pcCtr, *pc2Out);

    entityData->setData(pc2Out);
    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::getMinMax3D(pcCtr, min, max);
    log_info("Centroid to origin: New minimum is ("
             + std::to_string(min.x) + ", "
             + std::to_string(min.y) + ", "
             + std::to_string(min.z) + "), Maximum: ("
             + std::to_string(max.x) + ", "
             + std::to_string(max.y) + ", "
             + std::to_string(max.z) + ")");

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "use point of mass as origin of the pointcloud", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, &operate_ctr)
