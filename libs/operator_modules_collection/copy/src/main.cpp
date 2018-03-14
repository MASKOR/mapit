/*******************************************************************************
 *
 * Copyright 2016-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2016-2018 Tobias Neumann	<t.neumann@fh-aachen.de>
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
#include <mapit/operators/versioning/checkoutraw.h>
#include <mapit/operators/operationenvironment.h>
#include <mapit/errorcodes.h>
#include <mapit/operators/versioning/checkoutraw.h>
#include <mapit/depthfirstsearch.h>
#include "json11.hpp"

mapit::StatusCode copyEntity(mapit::OperationEnvironment* env, std::string source, std::string target, std::shared_ptr<mapit::msgs::Entity> srcEnt)
{
    if ( ! srcEnt ) {
        return MAPIT_STATUS_ERROR;
    }
    // Note: Only the entity is copied, it's new data is empty (because transient path is used to identify entities data)
    mapit::StatusCode s = env->getCheckout()->storeEntity(target, srcEnt);

    if(!mapitIsOk(s))
    {
        log_error("Could not copy \"" + source + "\" to \"" + target + "\"");
        return s;
    }
    else
    {
        std::shared_ptr<mapit::AbstractEntitydata> aedSource = env->getCheckout()->getEntitydataReadOnly(source);
        std::shared_ptr<mapit::AbstractEntitydata> aedTarget = env->getCheckout()->getEntitydataForReadWrite(target);
        mapit::istream *is = aedSource->startReadBytes();
        mapit::ostream *os = aedTarget->startWriteBytes();

        os->seekp(std::ios::beg);
        char buffer[1024];
        while(is->read(buffer, 1024)) {
            std::streamsize size=is->gcount();
            os->write(buffer, size);
        }
        std::streamsize size=is->gcount();
        if(size > 0)
        {
            os->write(buffer, size);
        }
        aedTarget->endWrite(os);
        aedSource->endRead(is);
    }
    return MAPIT_STATUS_OK;
}

mapit::StatusCode operate(mapit::OperationEnvironment* env)
{
    std::string jsonErr;
    json11::Json params = json11::Json::parse(env->getParameters(), jsonErr);

    if ( ! jsonErr.empty() ) {
        // can't parth json
        // TODO: good error msg
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }
    std::string source = params["source"].string_value();
    std::string target = params["target"].string_value();

    if(source.empty())
    {
        log_error("could not copy, source is not set");
    }
    if(target.empty())
    {
        log_error("could not copy, target is not set");
    }

    std::shared_ptr< mapit::msgs::Entity > srcEnt( env->getCheckout()->getEntity(source) );
    mapit::StatusCode s;
    if(srcEnt)
    {
        s = copyEntity(env, source, target, srcEnt);
    }
    else
    {
        std::shared_ptr< mapit::msgs::Tree > srcTree( env->getCheckout()->getTree(source) );
        if(srcTree)
        {
            ObjectReference nullRef;
            mapit::depthFirstSearch(
                        env->getCheckout(),
                        srcTree,
                        nullRef,
                        source,
                        depthFirstSearchAll(Commit),
                        depthFirstSearchAll(Commit),
                        depthFirstSearchAll(mapit::msgs::Tree),
                        depthFirstSearchAll(mapit::msgs::Tree),
                        [&](std::shared_ptr<mapit::msgs::Entity> obj, const ObjectReference& ref, const mapit::Path &path)
                        {
                            // create path for new entity => replace source with target in path
                            std::string pathNew = path;
                            size_t f = pathNew.find(source);
                            if (std::string::npos != f) {
                                pathNew.replace(f, source.length(), target);
                            } else {
                                log_error("error...");
                                s = MAPIT_STATUS_ERROR;
                                return false;
                            }

                            s = copyEntity(env, path, pathNew, obj);
                            if (!mapitIsOk(s)) {
                                return false;
                            }

                            return true;
                        },
                        depthFirstSearchAll(mapit::msgs::Entity)
                    );
        }
        else
        {
            s = MAPIT_STATUS_ERR_DB_NOT_FOUND;
            log_error("Path not found " + source);
        }
    }
    if(!mapitIsOk(s))
    {
        log_error("Could not copy \"" + source + "\" to \"" + target + "\"");
    }
    else
    {
        log_info("copied \"" + source + "\" to \"" + target + "\"");
    }

//    mapit::msgs::OperationDescription out;
//    out.set_operatorname(OPERATOR_NAME);
//    out.set_operatorversion(OPERATOR_VERSION);
//    OperationParameter *outTarget = out.add_params();
//    outTarget->set_key("target");
////    outTarget->set_mapval( map->id() );
////    outTarget->set_layerval( layer->id() );
////    outTarget->set_entityval( entity->id() );
//    OperationParameter *outMapname = out.add_params();
//    outMapname->set_key("leafsize");
//    outMapname->set_realval( leafSize );
//    env->setOutputDescription( out.SerializeAsString() );
    return MAPIT_STATUS_OK;
}

MAPIT_MODULE(OPERATOR_NAME, "copy tree or entity", "fhac", OPERATOR_VERSION, "any", &operate)
