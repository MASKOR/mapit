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
#include <QJsonDocument>
#include <QJsonObject>

using namespace mapit::msgs;

upns::StatusCode operate_ctr(upns::OperationEnvironment* env)
{
    QJsonDocument paramsDoc = QJsonDocument::fromJson( QByteArray(env->getParameters().c_str(), env->getParameters().length()) );
    QJsonObject params(paramsDoc.object());

    std::string target = params["target"].toString().toStdString();

    bool useAxisAlignedBoundingBox = params["useAABB"].toBool();
    bool genTf = params["genTf"].toBool();

    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    std::shared_ptr<PointcloudEntitydata> entityData = std::static_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
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
    if(!genTf)
    {
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
    }
    else
    {
        std::string tfEntityName = target + ".tf";
        // Get Target
        std::shared_ptr<mapit::msgs::Entity> tfEntity = env->getCheckout()->getEntity(tfEntityName);
        if(tfEntity == NULL)
        {
            // If target could not be received, create new entity
            // There must be an entity in order to write entity data in the next step
            tfEntity = std::shared_ptr<mapit::msgs::Entity>(new mapit::msgs::Entity);
            tfEntity->set_type(TfEntitydata::TYPENAME());
            StatusCode s = env->getCheckout()->storeEntity(tfEntityName, tfEntity);
            if(!upnsIsOk(s))
            {
                log_error("Failed to create transform entity.");
                return UPNS_STATUS_ERR_UNKNOWN;
            }
        }
        std::shared_ptr<AbstractEntitydata> abstractEntitydataTf = env->getCheckout()->getEntitydataForReadWrite( tfEntityName );
        if(abstractEntitydataTf == NULL)
        {
            return UPNS_STATUS_ERR_UNKNOWN;
        }
        std::shared_ptr<TfEntitydata> entityDataTf = std::static_pointer_cast<TfEntitydata>( abstractEntitydataTf );
        if(entityDataTf == NULL)
        {
            log_error("Tf Transform has wrong type.");
            return UPNS_STATUS_ERR_UNKNOWN;
        }
        TfMatPtr tf = TfMatPtr(new TfMat);
        Eigen::Affine3f transform(Eigen::Translation3f(-ctr[0], -ctr[1], -ctr[2]));
        *tf = transform.matrix();
        entityDataTf->setData(tf);
    }
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "use point of mass as origin of the pointcloud", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, &operate_ctr)
