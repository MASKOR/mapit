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
    std::cout << "DBG: START" << std::endl;
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

    std::cout << "DBG: RWEAD" << std::endl;
    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );

    std::shared_ptr<LASEntitydata> entityData = std::static_pointer_cast<LASEntitydata>(abstractEntitydata);
    liblas::Header header = reader.GetHeader();
    header.SetCompressed(false);
    std::unique_ptr<LASEntitydataWriter> writer = entityData->getWriter(header);
    writer->SetFilters(reader.GetFilters());
    writer->SetTransforms(reader.GetTransforms());
    //    std::copy(lasreader_iterator(reader),  lasreader_iterator(),
    //                      laswriter_iterator(writer));
    int i=0;
    std::cout << "DBG: DRLENG " << reader.GetHeader().GetDataRecordLength();
    while(reader.ReadNextPoint())
    {
        std::cout << "."; //"DBG: DRLENG " << reader.GetHeader().GetDataRecordLength();
        liblas::Point const&  current(reader.GetPoint());
        writer->WritePoint(current);
        ++i;
    }
    log_info("Points read:" + std::to_string(i));
    std::cout << "DBG: DONE " << std::to_string(i);
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Las File", "fhac", OPERATOR_VERSION, LASEntitydata_TYPENAME, &operate_load_las)
