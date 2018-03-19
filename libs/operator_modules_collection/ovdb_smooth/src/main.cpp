/*******************************************************************************
 *
 * Copyright      2017 Daniel Bulla	<d.bulla@fh-aachen.de>
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
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/operators/operationenvironment.h>
#include <mapit/operators/versioning/workspacewritable.h>
#include <mapit/layertypes/openvdblayer.h>
#include "openvdb/tools/LevelSetFilter.h"
#include <iostream>
#include <memory>
#include <mapit/errorcodes.h>
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

mapit::StatusCode operate_ovdb_smooth(mapit::OperationEnvironment* env)
{
    std::string jsonErr;
    json11::Json params = json11::Json::parse(env->getParameters(), jsonErr);
    if ( ! jsonErr.empty() ) {
        // can't parth json
        // TODO: good error msg
        return MAPIT_STATUS_INVALID_ARGUMENT;
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
            return MAPIT_STATUS_INVALID_ARGUMENT;
        }
    }
    if(output.empty())
    {
        output = params["target"].string_value();
        if(output.empty())
        {
            log_error("no output specified");
            return MAPIT_STATUS_INVALID_ARGUMENT;
        }
    }

    std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydataInput = env->getWorkspace()->getEntitydataReadOnly( input );
    if(!abstractEntitydataInput)
    {
        log_error("input does not exist or is not readable.");
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }
    std::shared_ptr<FloatGridEntitydata> entityDataInput = std::dynamic_pointer_cast<FloatGridEntitydata>( abstractEntitydataInput );
    if(entityDataInput == nullptr)
    {
        log_error("Wrong type");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    FloatGridPtr inputGrid = entityDataInput->getData();

    std::shared_ptr<Entity> ent = env->getWorkspace()->getEntity(output);
    if(ent)
    {
        log_info("Output grid already exists. ignoring voxelsize.");
        std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydataOutput = env->getWorkspace()->getEntitydataReadOnly( output );
        if(!abstractEntitydataOutput)
        {
            log_error("could not read output grid");
            return MAPIT_STATUS_INVALID_ARGUMENT;
        }
        std::shared_ptr<FloatGridEntitydata> entityDataOutput = std::dynamic_pointer_cast<FloatGridEntitydata>( abstractEntitydataOutput );
        if(!entityDataOutput)
        {
            log_error("could not cast output to FloatGrid");
            return MAPIT_STATUS_INVALID_ARGUMENT;
        }
    }
    else
    {
        ent = std::shared_ptr<Entity>(new Entity);
        ent->set_type(FloatGridEntitydata::TYPENAME());
        mapit::StatusCode s = env->getWorkspace()->storeEntity(output, ent);
        if(!mapitIsOk(s))
        {
            log_error("Failed to create entity.");
            return MAPIT_STATUS_ERR_DB_IO_ERROR;
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

    std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydataOutput = env->getWorkspace()->getEntitydataForReadWrite( output );
    if(!abstractEntitydataOutput)
    {
        log_error("could not read output asset");
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }
    std::shared_ptr<FloatGridEntitydata> entityDataOutput = std::dynamic_pointer_cast<FloatGridEntitydata>( abstractEntitydataOutput );
    if(!entityDataOutput)
    {
        log_error("could not cast output to FloatGrid");
        return MAPIT_STATUS_INVALID_ARGUMENT;
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
    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME, "Loads a Pcd File", "fhac", OPERATOR_VERSION, FloatGridEntitydata_TYPENAME, &operate_ovdb_smooth)
