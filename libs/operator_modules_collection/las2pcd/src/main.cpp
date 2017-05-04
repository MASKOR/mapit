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
    std::cout << "DBG: BEGINNING" << std::endl;
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
    std::shared_ptr<AbstractEntitydata> abstractEntitydataInput = env->getCheckout()->getEntitydataReadOnly( target );
    if(!abstractEntitydataInput)
    {
        log_error("input does not exist ore is not readable.");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    pcl::PointCloud<pcl::PointXYZINormal> pc;
    {
        std::cout << "DBG: READ" << std::endl;
        std::shared_ptr<LASEntitydata> entityDataLASInput = std::static_pointer_cast<LASEntitydata>( abstractEntitydataInput );
        std::unique_ptr<LASEntitydataReader> reader = entityDataLASInput->getReader();

        const liblas::Header header = reader->GetHeader();
        uint32_t points = header.GetPointRecordsCount();

        std::cout << "DBG: POINTS" << points << std::endl;
        pc.points.reserve(points);
        uint32_t i=0;
        while(reader->ReadNextPoint())
        {
            liblas::Point const& point = reader->GetPoint();
            if(i%1000 == 0) {
                std::cout << "." << point.GetX() << " " << point.GetRawY() << "; " ; //DBG
            }
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
        log_info("Looped " + std::to_string(i) + "points. Resulting PCD has " + std::to_string(points) + " points.");
        //dispose reader
        std::cout << "DBG: Looped" << std::to_string(i) << std::endl;
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

    std::cout << "DBG: WRITE" << std::endl;
    entityDataPCLOutput->setData(cloud);
    std::cout << "DBG: DONE" << std::endl;

    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Las File", "fhac", OPERATOR_VERSION, LASEntitydata_TYPENAME, &operate_pcd2las)
