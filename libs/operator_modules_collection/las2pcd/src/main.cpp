#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/layertypes/lastype.h>
#include <upns/layertypes/lasentitydatawriter.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <iostream>
#include <memory>
#include <upns/errorcodes.h>
#include "json11.hpp"
#include "liblas/liblas.hpp"
#include <fstream>
#include <iomanip>

upns::StatusCode operate_pcd2las(upns::OperationEnvironment* env)
{
    std::string jsonErr;
    json11::Json params = json11::Json::parse(env->getParameters(), jsonErr);
    if ( ! jsonErr.empty() ) {
        // can't parse json
        // TODO: good error msg
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    std::string target = params["target"].string_value();
    if(target.empty())
    {
        log_error("no target specified");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    bool demean = params["demean"].bool_value();
    bool normalize = params["normalize"].bool_value();
    double normalizeScale = params["normalizeScale"].number_value();
    if(normalizeScale < 0.001)
    {
        if(normalizeScale < 0.0)
        {
            log_error("normalizeScale was negative");
            return UPNS_STATUS_INVALID_ARGUMENT;
        }
        normalizeScale = 10.0; // 10m is default
    }
    else if(!normalize)
    {
        if(normalizeScale != 1.0)
        {
            log_warn("normalizeScale was set, but normalize was not active");
        }
        normalizeScale = 1.0;
    }

    std::cout << "normalize: " << normalizeScale << std::endl;

    std::shared_ptr<AbstractEntitydata> abstractEntitydataInput = env->getCheckout()->getEntitydataReadOnly( target );
    if(!abstractEntitydataInput)
    {
        log_error("input does not exist ore is not readable.");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    pcl::PointCloud<pcl::PointXYZINormal> pc;
    {
        std::shared_ptr<LASEntitydata> entityDataLASInput = std::static_pointer_cast<LASEntitydata>( abstractEntitydataInput );
        std::unique_ptr<LASEntitydataReader> reader = entityDataLASInput->getReader();

        const liblas::Header header = reader->GetHeader();
        uint32_t points = header.GetPointRecordsCount();

        pc.points.reserve(points);
        uint32_t i=0;
        if(demean || normalize)
        {
            double minX = header.GetMinX();
            double minY = header.GetMinY();
            double minZ = header.GetMinZ();
            double maxX = header.GetMaxX();
            double maxY = header.GetMaxY();
            double maxZ = header.GetMaxZ();

            double sx = maxX-minX;
            double sy = maxY-minY;
            double sz = maxZ-minZ;
            double maxDim = std::max(std::max(sx, sy), sz);
            double scale = normalizeScale / maxDim;

            double cx = minX+sx*0.5;
            double cy = minY+sy*0.5;
            double cz = minZ+sz*0.5;
            while(reader->ReadNextPoint())
            {
                liblas::Point current(reader->GetPoint());
                pcl::PointXYZINormal pclPoint;
                pclPoint.x = (current.GetX()-cx)*scale;
                pclPoint.y = (current.GetY()-cy)*scale;
                pclPoint.z = (current.GetZ()-cz)*scale;
                if(current.GetData().size() >= 3)
                {
                    pclPoint.normal_x = current.GetData().at(0);
                    pclPoint.normal_y = current.GetData().at(1);
                    pclPoint.normal_z = current.GetData().at(2);
                }
                pclPoint.intensity = current.GetIntensity();
                pc.points.push_back(pclPoint);
                ++i;
            }
        }
        else
        {
            while(reader->ReadNextPoint())
            {
                liblas::Point const& point = reader->GetPoint();
                pcl::PointXYZINormal pclPoint;
                pclPoint.x = point.GetX();
                pclPoint.y = point.GetY();
                pclPoint.z = point.GetZ();
                if(point.GetData().size() >= 3)
                {
                    pclPoint.normal_x = point.GetData().at(0);
                    pclPoint.normal_y = point.GetData().at(1);
                    pclPoint.normal_z = point.GetData().at(2);
                }
                pclPoint.intensity = point.GetIntensity();
                pc.points.push_back(pclPoint);
                i++;
            }
        }
        log_info("Looped " + std::to_string(i) + "points. Resulting PCD has " + std::to_string(points) + " points.");
        //dispose reader
    }
    std::shared_ptr<mapit::msgs::Entity> pclEntity(new mapit::msgs::Entity);
    pclEntity->set_type(PointcloudEntitydata::TYPENAME());
    StatusCode s = env->getCheckout()->storeEntity(target, pclEntity);
    if(!upnsIsOk(s))
    {
        log_error("Failed to create entity.");
    }
    std::shared_ptr<AbstractEntitydata> abstractEntitydataOutput = env->getCheckout()->getEntitydataForReadWrite( target );
    std::shared_ptr<PointcloudEntitydata> entityDataPCLOutput = std::static_pointer_cast<PointcloudEntitydata>( abstractEntitydataOutput );

    std::shared_ptr<pcl::PCLPointCloud2> cloud(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(pc, *cloud);

    entityDataPCLOutput->setData(cloud);

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Las File", "fhac", OPERATOR_VERSION, LASEntitydata_TYPENAME, &operate_pcd2las)
