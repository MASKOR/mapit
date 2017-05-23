#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/layertypes/lastype.h>
#include <upns/layertypes/lasentitydatawriter.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <iostream>
#include <memory>
#include <upns/errorcodes.h>
#include "json11.hpp"
#include "liblas/liblas.hpp"
#include <fstream>

upns::StatusCode operate_load_las(upns::OperationEnvironment* env)
{
    std::string jsonErr;
    json11::Json params = json11::Json::parse(env->getParameters(), jsonErr);

    if ( ! jsonErr.empty() ) {
        // can't parse json
        // TODO: good error msg
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    liblas::ReaderFactory f;
    std::string filename = params["filename"].string_value();
    if(filename.empty())
    {
        log_error("parameter \"filename\" missing");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
// demean and normalize not supported!
// Reason: This would require to rebuild the read LAS datastructure and create custom points.
// Points may vary in dataformat which might result in multiple codepaths.
// Pcds can be demeaned.
//    bool demean = params["demean"].bool_value();
//    bool normalize = params["normalize"].bool_value();
//    double normalizeScale = params["normalizeScale"].number_value();
//    if(normalizeScale < 0.001)
//    {
//        if(normalizeScale < 0.0)
//        {
//            log_error("normalizeScale was negative");
//            return UPNS_STATUS_INVALID_ARGUMENT;
//        }
//        normalizeScale = 10.0; // 10m is default
//    }
//    else if(!normalize)
//    {
//        if(normalizeScale != 1.0)
//        {
//            log_warn("normalizeScale was set, but normalize was not active");
//        }
//        normalizeScale = 1.0;
//    }

    std::ifstream ifs;
    ifs.open(filename, std::ifstream::in);
    if ( ! ifs.good() )
    {
        log_error("Couldn't read file" + filename);
        return UPNS_STATUS_FILE_NOT_FOUND;
    }

    std::string target = params["target"].string_value();

    std::shared_ptr<mapit::msgs::Entity> pclEntity(new mapit::msgs::Entity);
    pclEntity->set_type(LASEntitydata::TYPENAME());
    StatusCode s = env->getCheckout()->storeEntity(target, pclEntity);
    if(!upnsIsOk(s))
    {
        log_error("Failed to create entity.");
    }
    liblas::Reader reader = f.CreateWithStream(ifs);

    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );

    std::shared_ptr<LASEntitydata> entityData = std::static_pointer_cast<LASEntitydata>(abstractEntitydata);
    liblas::Header header(reader.GetHeader());
    header.SetCompressed(false);
    std::unique_ptr<LASEntitydataWriter> writer = entityData->getWriter(header);
    writer->SetFilters(reader.GetFilters());
    writer->SetTransforms(reader.GetTransforms());
    //    std::copy(lasreader_iterator(reader),  lasreader_iterator(),
    //                      laswriter_iterator(writer));
    int i=0;
//    if(demean || normalize)
//    {
//        double minX = reader.GetHeader().GetMinX();
//        double minY = reader.GetHeader().GetMinY();
//        double minZ = reader.GetHeader().GetMinZ();
//        double maxX = reader.GetHeader().GetMaxX();
//        double maxY = reader.GetHeader().GetMaxY();
//        double maxZ = reader.GetHeader().GetMaxZ();

//        double sx = maxX-minX;
//        double sy = maxY-minY;
//        double sz = maxZ-minZ;
//        double maxDim = std::max(std::max(sx, sy), sz);
//        double scale = normalizeScale / maxDim;

//        double cx = minX+sx*0.5;
//        double cy = minY+sy*0.5;
//        double cz = minZ+sz*0.5;

//        while(reader.ReadNextPoint())
//        {
//            liblas::Point current(reader.GetPoint());
//            liblas::Point newpoint(&reader.GetHeader());
//            newpoint.SetX((current.GetX()-cx)*scale);
//            newpoint.SetY((current.GetY()-cy)*scale);
//            newpoint.SetZ((current.GetZ()-cz)*scale);
//            writer->WritePoint(newpoint);
//            ++i;
//        }
//    }
//    else
//    {
        while(reader.ReadNextPoint())
        {
            liblas::Point const&  current(reader.GetPoint());

            writer->WritePoint(current);
            ++i;
        }
//    }
    log_info("Points read:" + std::to_string(i));
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Las File", "fhac", OPERATOR_VERSION, LASEntitydata_TYPENAME, &operate_load_las)
