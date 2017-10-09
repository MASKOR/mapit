#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/errorcodes.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <QJsonDocument>
#include <QJsonObject>
#include <pcl/filters/radius_outlier_removal.h>

void radiusOutlierRemoval(pcl::PCLPointCloud2 const &original,
                          pcl::PCLPointCloud2 &filtered,
                          std::double_t const radius,
                          std::int32_t const minimumNeighbors)
{
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(pcl::PCLPointCloud2ConstPtr(new pcl::PCLPointCloud2(original)));
    filter.setMinNeighborsInRadius(minimumNeighbors);
    filter.setRadiusSearch(radius);
    filter.filter(filtered);
    return;
}

upns::StatusCode operateRadiusOutlierRemoval(upns::OperationEnvironment* environment)
{
    log_info("┌filter: radius outlier removal");
    QByteArray parametersRaw(environment->getParameters().c_str(),
                             environment->getParameters().length());
    QJsonObject parameters(QJsonDocument::fromJson(parametersRaw).object());
    log_info("├─┬parameters");

    std::string source = parameters["source"].toString().toStdString();
    if (source.empty()) {
        log_error("└─┴─source entity string is empty");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    log_info("│ ├─source: '" << source << "'");

    std::string target = parameters["target"].toString().toStdString();
    if (source.empty()) {
        log_error("└─┴─target entity string is empty");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    log_info("│ ├─target: '" << target << "'");

    std::double_t radius = parameters["radius"].toDouble();
    if (radius <= 0.0) {
        log_error("└─┴─radius is smaller than or equal to zero");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    log_info("│ ├─radius: " << radius);

    std::int32_t minimumNeighbors = parameters["minimumNeighbors"].toInt();
    if (minimumNeighbors <= 0) {
        log_error("└─┴─minimum neighbors is smaller than or equal to zero");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    log_info("│ └─minimum neighbors: " << minimumNeighbors);

    log_info("├─┬load original point cloud...");
    std::shared_ptr<PointcloudEntitydata> sourceData =
            std::dynamic_pointer_cast<PointcloudEntitydata>(environment->getCheckout()->getEntitydataForReadWrite(source));
    if (sourceData == nullptr) {
        log_error("└─┴─source entity is no point cloud");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    std::shared_ptr<pcl::PCLPointCloud2> original = sourceData->getData();
    log_info("│ ├─fields:");
    std::for_each(original->fields.begin(),
                  original->fields.end(),
                  [](pcl::PCLPointField const &field) {
        log_info("│ │        " << field.name);
        return;
    });
    log_info("│ ├─size: " << original->height * original->width);
    log_info("│ └─dense: " << (original->is_dense ? "true" : "false"));

    log_info("├─┬start filtering point cloud...");
    std::shared_ptr<pcl::PCLPointCloud2> filtered(new pcl::PCLPointCloud2);
    try {
        radiusOutlierRemoval(*original, *filtered, radius, minimumNeighbors);
    } catch(...) {
        log_error("└─┴─filtering failed");
        return UPNS_STATUS_ERR_UNKNOWN;
    }
    log_info("│ └─size: " << filtered->height * filtered->width);

    log_info("└─┬save filtered point cloud...");
    std::shared_ptr<mapit::msgs::Entity> targetEntity = environment->getCheckout()->getEntity(target);
    if (targetEntity == NULL) {
        targetEntity = std::shared_ptr<mapit::msgs::Entity>(new mapit::msgs::Entity);
        targetEntity->set_type(PointcloudEntitydata::TYPENAME());
        if (!upnsIsOk(environment->getCheckout()->storeEntity(target, targetEntity))) {
            log_error("  └─failed to create target entity");
            return UPNS_STATUS_ERR_UNKNOWN;
        }
        log_info("  ├─created new entity '" << target << "'");
    }
    std::shared_ptr<PointcloudEntitydata> targetData =
            std::dynamic_pointer_cast<PointcloudEntitydata>(environment->getCheckout()->getEntitydataForReadWrite(target));
    if (targetData == NULL) {
        log_error("  └─target entity is no point cloud");
        return UPNS_STATUS_ERR_UNKNOWN;
    }
    targetData->setData(filtered);
    log_info("  └─complete");

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME,
            "filter: radius outlier removal",
            "Marcus Meeßen",
            OPERATOR_VERSION,
            PointcloudEntitydata_TYPENAME,
            &operateRadiusOutlierRemoval)
