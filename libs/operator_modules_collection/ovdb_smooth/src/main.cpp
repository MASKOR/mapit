#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/layertypes/openvdblayer.h>
#include "openvdb/tools/LevelSetFilter.h"
#include <iostream>
#include <memory>
#include <upns/errorcodes.h>
#include "json11.hpp"

using namespace mapit::msgs;

struct PrintInterrupter
{
    int counter = 0;
    PrintInterrupter () {}
    void start(const char* name = NULL) {
        std::cout << name;
        std::cout.rdbuf()->pubsetbuf(0, 0);
    }
    void end() {}
    inline bool wasInterrupted(int percent = -1) {
        counter++;
        if(counter%20==1)
        {
            std::cout << "." << std::flush;
        }
        if(counter%200==1)
        {
            std::cout << percent << "%" << std::endl;
        }
        return false;
    }
};

upns::StatusCode operate_ovdb_smooth(upns::OperationEnvironment* env)
{
    std::string jsonErr;
    json11::Json params = json11::Json::parse(env->getParameters(), jsonErr);
    if ( ! jsonErr.empty() ) {
        // can't parth json
        // TODO: good error msg
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    double dilateerode = params["radius"].number_value();

    if(dilateerode == 0.0)
    {
        dilateerode = 0.01f;
    }

    double voxelsize = params["voxelsize"].number_value();

    if(voxelsize == 0.0)
    {
        voxelsize = 0.01f;
    }

    double smoothness = params["smoothness"].number_value();

    std::string input =  params["input"].string_value();
    std::string output = params["output"].string_value();
    if(input.empty())
    {
        input = params["target"].string_value();
        if(input.empty())
        {
            log_error("no input specified");
            return UPNS_STATUS_INVALID_ARGUMENT;
        }
    }
    if(output.empty())
    {
        output = params["target"].string_value();
        if(output.empty())
        {
            log_error("no output specified");
            return UPNS_STATUS_INVALID_ARGUMENT;
        }
    }

    std::shared_ptr<AbstractEntitydata> abstractEntitydataInput = env->getCheckout()->getEntitydataReadOnly( input );
    if(!abstractEntitydataInput)
    {
        log_error("input does not exist ore is not readable.");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    std::shared_ptr<FloatGridEntitydata> entityDataInput = std::dynamic_pointer_cast<FloatGridEntitydata>( abstractEntitydataInput );
    if(entityDataInput == nullptr)
    {
        log_error("Wrong type");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    upnsFloatGridPtr inputGrid = entityDataInput->getData();

    std::shared_ptr<Entity> ent = env->getCheckout()->getEntity(output);
    if(ent)
    {
        log_info("Output grid already exists. ignoring voxelsize.");
        std::shared_ptr<AbstractEntitydata> abstractEntitydataOutput = env->getCheckout()->getEntitydataReadOnly( output );
        if(!abstractEntitydataOutput)
        {
            log_error("could not read output grid");
            return UPNS_STATUS_INVALID_ARGUMENT;
        }
        std::shared_ptr<FloatGridEntitydata> entityDataOutput = std::dynamic_pointer_cast<FloatGridEntitydata>( abstractEntitydataOutput );
        if(!entityDataOutput)
        {
            log_error("could not cast output to FloatGrid");
            return UPNS_STATUS_INVALID_ARGUMENT;
        }
    }
    else
    {
        ent = std::shared_ptr<Entity>(new Entity);
        ent->set_type(FloatGridEntitydata::TYPENAME());
        StatusCode s = env->getCheckout()->storeEntity(output, ent);
        if(!upnsIsOk(s))
        {
            log_error("Failed to create entity.");
            return UPNS_STATUS_ERR_DB_IO_ERROR;
        }
    }
    PrintInterrupter interr;
    openvdb::tools::LevelSetFilter<openvdb::FloatGrid, openvdb::FloatGrid::template ValueConverter<float>::Type, PrintInterrupter> filter(*inputGrid, &interr);
    filter.offset(-dilateerode);
    std::cout << "Dilation finished";
    filter.gaussian(smoothness);
    std::cout << "Gauss finished";
    filter.offset(dilateerode);
    std::cout << "Erosion finished";

    std::shared_ptr<AbstractEntitydata> abstractEntitydataOutput = env->getCheckout()->getEntitydataForReadWrite( output );
    if(!abstractEntitydataOutput)
    {
        log_error("could not read output asset");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    std::shared_ptr<FloatGridEntitydata> entityDataOutput = std::dynamic_pointer_cast<FloatGridEntitydata>( abstractEntitydataOutput );
    if(!entityDataOutput)
    {
        log_error("could not cast output to FloatGrid");
        return UPNS_STATUS_INVALID_ARGUMENT;
    }
    entityDataOutput->setData(inputGrid);

//    OperationDescription out;
//    out.set_operatorname(OPERATOR_NAME);
//    out.set_operatorversion(OPERATOR_VERSION);
//    OperationParameter *outTarget = out.add_params();
//    outTarget->set_key("target");
////    outTarget->set_mapval( map->id() );
////    outTarget->set_layerval( layer->id() );
////    outTarget->set_entityval( entity->id() );
//    OperationParameter *outMapname = out.add_params();
//    outMapname->set_key("mapname");
//    outMapname->set_strval( map->name() );
//    env->setOutputDescription( out.SerializeAsString() );
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "Loads a Pcd File", "fhac", OPERATOR_VERSION, FloatGridEntitydata_TYPENAME, &operate_ovdb_smooth)
