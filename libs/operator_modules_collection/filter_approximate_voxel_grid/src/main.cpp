#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/errorcodes.h>
#include <QJsonDocument>
#include <QJsonObject>
#include <pcl/filters/approximate_voxel_grid.h>

std::map<std::string, std::uint16_t const> const FIELD = {
    {"x",         0x0001},
    {"y",         0x0002},
    {"z",         0x0004},
    {"r",         0x0008},
    {"g",         0x000F},
    {"b",         0x0010},
    {"a",         0x0020},
    {"normal_x",  0x0040},
    {"normal_y",  0x0080},
    {"normal_z",  0x00F0},
    {"curvature", 0x0100},
    {"intensity", 0x0200},
    {"strength",  0x0400},
    {"", 0xFFFF},
};

template<typename T>
void approximateVoxelGrid(pcl::PCLPointCloud2 const &original,
                          pcl::PCLPointCloud2 &filtered,
                          std::double_t const leafSize,
                          bool const downsampleAllData)
{
    pcl::ApproximateVoxelGrid<T> filter;
    pcl::PointCloud<T> originalT;
    pcl::PointCloud<T> filteredT;
    pcl::fromPCLPointCloud2(original, originalT);
    filter.setInputCloud(originalT.makeShared());
    filter.setDownsampleAllData(downsampleAllData);
    filter.setLeafSize(leafSize, leafSize, leafSize);
    filter.filter(filteredT);
    pcl::toPCLPointCloud2(filteredT, filtered);
    return;
}

upns::StatusCode operateApproximateVoxelGrid(upns::OperationEnvironment* environment)
{
    log_info("┌filter: approximate voxel grid");
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

    std::double_t leafSize = parameters["leafSize"].toDouble();
    if (leafSize <= 0.0) {
        log_error("└─┴─leaf size is smaller than or equal to zero");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    log_info("│ ├─leaf size: " << leafSize);

    bool downsampleAllData = parameters["downsampleAllData"].toBool();
    log_info("│ └─downsample all data: " << (downsampleAllData ? "true" : "false"));

    log_info("├─┬load original point cloud...");
    std::shared_ptr<PointcloudEntitydata> sourceData =
            std::dynamic_pointer_cast<PointcloudEntitydata>(environment->getCheckout()->getEntitydataForReadWrite(source));
    if (sourceData == nullptr) {
        log_error("└─┴─source entity is no point cloud");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    std::shared_ptr<pcl::PCLPointCloud2> original = sourceData->getData();
    log_info("│ ├─fields:");
    std::uint16_t fields = 0;
    std::for_each(original->fields.begin(),
                  original->fields.end(),
                  [&fields](pcl::PCLPointField const &field) {
        fields |= FIELD.find(field.name) != FIELD.end()
                ? FIELD.at(field.name)
                : FIELD.at("");
        log_info("│ │        " << field.name);
        return;
    });
    log_info("│ ├─size: " << original->height * original->width);
    log_info("│ └─dense: " << (original->is_dense ? "true" : "false"));

    log_info("├─┬detect point type...");
    void (*func)(pcl::PCLPointCloud2 const &,
                 pcl::PCLPointCloud2 &,
                 std::double_t const,
                 bool const);
    std::uint16_t const xyz = (FIELD.at("x") | FIELD.at("y") | FIELD.at("z"));
    std::uint16_t const rgb = (FIELD.at("r") | FIELD.at("g") | FIELD.at("b"));
    std::uint16_t const normal = (FIELD.at("normal_x") | FIELD.at("normal_y") | FIELD.at("normal_z"));
    if (fields == xyz) {
        log_info("│ └─detected type: PointXYZ");
        func = &approximateVoxelGrid<pcl::PointXYZ>;
    } else if (fields == (xyz | FIELD.at("intensity"))) {
        log_info("│ └─detected type: PointXYZI");
        func = &approximateVoxelGrid<pcl::PointXYZI>;
    } else if (fields == (xyz | normal | FIELD.at("curvature"))) {
        log_info("│ └─detected type: PointNormal");
        func = &approximateVoxelGrid<pcl::PointNormal>;
    } else if (fields == (xyz | FIELD.at("intensity") | normal | FIELD.at("curvature"))) {
        log_info("│ └─detected type: PointXYZINormal");
        func = &approximateVoxelGrid<pcl::PointXYZINormal>;
    } else if (fields == (xyz | rgb)) {
        log_info("│ └─detected type: PointXYZRGB");
        func = &approximateVoxelGrid<pcl::PointXYZRGB>;
    } else if (fields == (xyz | rgb | FIELD.at("a"))) {
        log_info("│ └─detected type: PointXYZRGBA");
        func = &approximateVoxelGrid<pcl::PointXYZRGBA>;
    } else if (fields == (xyz | rgb | normal | FIELD.at("curvature"))) {
        log_info("│ └─detected type: PointXYZRGBNormal");
        func = &approximateVoxelGrid<pcl::PointXYZRGBNormal>;
    } else if (fields == (xyz | FIELD.at("strength"))) {
        log_info("│ └─detected type: InterestPoint");
        func = &approximateVoxelGrid<pcl::InterestPoint>;
    } else {
        log_error("└─┴─unknown point cloud type");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }

    log_info("├─┬start filtering point cloud...");
    std::shared_ptr<pcl::PCLPointCloud2> filtered(new pcl::PCLPointCloud2);
    try {
        func(*original, *filtered, leafSize, downsampleAllData);
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
            std::dynamic_pointer_cast<PointcloudEntitydata>(environment->getCheckout()->getEntitydataForReadWrite(source));
    if (targetData == NULL) {
        log_error("  └─target entity is no point cloud");
        return UPNS_STATUS_ERR_UNKNOWN;
    }
    targetData->setData(filtered);
    log_info("  └─complete");

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME,
            "filter: approximate voxel grid",
            "Marcus Meeßen",
            OPERATOR_VERSION,
            PointcloudEntitydata_TYPENAME,
            &operateApproximateVoxelGrid)
