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
    std::unique_ptr<LASEntitydataWriter> writer = entityData->getWriter(reader.GetHeader());
    int i=0;
    while(reader.ReadNextPoint())
    {
        liblas::Point const&  current(reader.GetPoint());
        writer->WritePoint(current);
        ++i;
    }
    log_info("Points read:" + std::to_string(i));
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Las File", "fhac", OPERATOR_VERSION, LASEntitydata_TYPENAME, &operate_load_las)
