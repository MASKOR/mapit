/*******************************************************************************
 *
 * Copyright      2017 Marcus Meeßen	<marcus.meessen@alumni.fh-aachen.de>
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
#include <mapit/layertypes/pointcloudlayer.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>
#include <mapit/errorcodes.h>
#include <QJsonDocument>
#include <QJsonObject>
#include <pcl/filters/statistical_outlier_removal.h>

void statisticalOutlierRemoval(pcl::PCLPointCloud2 const &original,
                               pcl::PCLPointCloud2 &filtered,
                               std::double_t const deviation,
                               std::int32_t const neighbors)
{
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(pcl::PCLPointCloud2ConstPtr(new pcl::PCLPointCloud2(original)));
    filter.setMeanK(neighbors);
    filter.setStddevMulThresh(deviation);
    filter.filter(filtered);
    return;
}

mapit::StatusCode operateStatisticalOutlierRemoval(mapit::OperationEnvironment* environment)
{
    log_info("┌filter: statistical outlier removal");
    QByteArray parametersRaw(environment->getParameters().c_str(),
                             environment->getParameters().length());
    QJsonObject parameters(QJsonDocument::fromJson(parametersRaw).object());
    log_info("├─┬parameters");

    std::string source = parameters["source"].toString().toStdString();
    if (source.empty()) {
        log_error("└─┴─source entity string is empty");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    log_info("│ ├─source: '" << source << "'");

    std::string target = parameters["target"].toString().toStdString();
    if (source.empty()) {
        log_error("└─┴─target entity string is empty");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    log_info("│ ├─target: '" << target << "'");

    std::double_t deviation = parameters["deviation"].toDouble();
    if (std::isnan(deviation) || std::isinf(deviation)) {
        log_error("└─┴─deviation has no valid value");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    log_info("│ ├─deviation: " << deviation << "σ");

    std::int32_t neighbors = parameters["neighbors"].toInt();
    if (neighbors <= 0) {
        log_error("└─┴─neighbors is smaller than or equal to zero");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    log_info("│ └─neighbors: " << neighbors);

    log_info("├─┬load original point cloud...");
    std::shared_ptr<PointcloudEntitydata> sourceData =
            std::dynamic_pointer_cast<PointcloudEntitydata>(environment->getWorkspace()->getEntitydataForReadWrite(source));
    if (sourceData == nullptr) {
        log_error("└─┴─source entity is no point cloud");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
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
        statisticalOutlierRemoval(*original, *filtered, deviation, neighbors);
    } catch(...) {
        log_error("└─┴─filtering failed");
        return MAPIT_STATUS_ERR_UNKNOWN;
    }
    log_info("│ └─size: " << filtered->height * filtered->width);

    log_info("└─┬save filtered point cloud...");
    std::shared_ptr<mapit::msgs::Entity> targetEntity = environment->getWorkspace()->getEntity(target);
    if (targetEntity == NULL) {
        targetEntity = std::shared_ptr<mapit::msgs::Entity>(new mapit::msgs::Entity);
        targetEntity->set_type(PointcloudEntitydata::TYPENAME());
        if (!mapitIsOk(environment->getWorkspace()->storeEntity(target, targetEntity))) {
            log_error("  └─failed to create target entity");
            return MAPIT_STATUS_ERR_UNKNOWN;
        }
        log_info("  ├─created new entity '" << target << "'");
    }
    std::shared_ptr<PointcloudEntitydata> targetData =
            std::dynamic_pointer_cast<PointcloudEntitydata>(environment->getWorkspace()->getEntitydataForReadWrite(target));
    if (targetData == NULL) {
        log_error("  └─target entity is no point cloud");
        return MAPIT_STATUS_ERR_UNKNOWN;
    }
    targetData->setData(filtered);
    log_info("  └─complete");

    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME,
            "filter: statistical outlier removal",
            "Marcus Meeßen",
            OPERATOR_VERSION,
            PointcloudEntitydata_TYPENAME,
            &operateStatisticalOutlierRemoval)
