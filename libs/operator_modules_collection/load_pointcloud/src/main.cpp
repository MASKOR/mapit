/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2017 Tobias Neumann	<t.neumann@fh-aachen.de>
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
#include <mapit/operators/versioning/checkoutraw.h>
#include <mapit/operators/operationenvironment.h>
#include <mapit/operators/versioning/checkoutraw.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <memory>
#include <mapit/errorcodes.h>
//#include "param.pb.h"
#include "json11.hpp"

using namespace mapit::msgs;

bool field_present(const std::string& name, const  std::vector<pcl::PCLPointField>& flist){
    for(int i=0; i<flist.size(); i++){
        if (flist[i].name == name) return true;
    }
    return false;
}

void chooseDefaultRepresentation ( const std::vector<pcl::PCLPointField>& flist )
{
    if ( field_present("rgba", flist)){
        //return new PointCloudRGBFactory<pcl::PointXYZRGBA, pcl::RGB>();
    } else if ( field_present("rgb", flist)){
            //return new PointCloudRGBFactory<pcl::PointXYZRGB, pcl::RGB>();
    } else if ( field_present("intensity", flist)){
        //return NULL; //PointCloudIFactory<pcl::PointXYZI, pcl::Intensity>;
    } else if ( field_present("label", flist)){
        //return new PointCloudLabelFactory<pcl::PointXYZ, pcl::Label>();
    }
    //return new PointCloudCRangeFactory<>("z");
}

mapit::StatusCode operate_load_pointcloud(mapit::OperationEnvironment* env)
{
    upnsPointcloud2Ptr pc2( new pcl::PCLPointCloud2);

    std::string jsonErr;
    json11::Json params = json11::Json::parse(env->getParameters(), jsonErr);

    if ( ! jsonErr.empty() ) {
        // can't parse json
        // TODO: good error msg
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }

    pcl::PCDReader reader;
    std::string filename = params["filename"].string_value();
    if(filename.empty())
    {
        log_error("parameter \"filename\" missing");
        return MAPIT_STATUS_INVALID_ARGUMENT;
    }
    if ( reader.read(filename, *pc2) < 0 )
    {
        log_error("Couldn't read file" + filename);
        return MAPIT_STATUS_FILE_NOT_FOUND;
    }

    std::string target = params["target"].string_value();

    unsigned long sec = params["sec"].int_value();
    unsigned long nsec = params["nsec"].int_value();
    std::shared_ptr<Entity> pclEntity(new Entity);
    pclEntity->set_type(PointcloudEntitydata::TYPENAME());
    pclEntity->set_frame_id( params["frame_id"].string_value() );
    pclEntity->mutable_stamp()->set_sec( sec );
    pclEntity->mutable_stamp()->set_nsec( nsec );
    mapit::StatusCode s = env->getCheckout()->storeEntity(target, pclEntity);
    if(!upnsIsOk(s))
    {
        log_error("Failed to create entity.");
    }
    std::shared_ptr<mapit::AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );

    std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>(abstractEntitydata);
    if(entityData == nullptr)
    {
        log_error("Wrong type");
        return MAPIT_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    entityData->setData( pc2 );

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

MAPIT_MODULE(OPERATOR_NAME, "Loads a Pcd File", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, &operate_load_pointcloud)
