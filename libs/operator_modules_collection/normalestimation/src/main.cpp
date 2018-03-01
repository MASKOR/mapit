/*******************************************************************************
 *
 * Copyright 2015-2017 Daniel Bulla	<d.bulla@fh-aachen.de>
 *           2015-2016 Tobias Neumann	<t.neumann@fh-aachen.de>
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

#include <upns/operators/module.h>
#include <upns/layertypes/pointcloudlayer.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/logging.h>
#include <iostream>
#include <pcl/features/normal_3d.h>
#include <memory>
#include <upns/errorcodes.h>
#include "json11.hpp"
#include <pcl/search/search.h>

upns::StatusCode operate(upns::OperationEnvironment* env)
{
    std::string jsonErr;
    json11::Json params = json11::Json::parse(env->getParameters(), jsonErr);
    if ( ! jsonErr.empty() ) {
        // can't parth json
        // TODO: good error msg
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    bool useK = true;
    double radius = params["radius"].number_value();
    if(radius == 0.0)
    {
        radius = 0.5f;
    }
    else
    {
        useK = false;
    }
    int k = params["k"].number_value();
    if(k == 0)
    {
        k = 5;
    }
    else
    {
        useK = true;
    }

    std::string target = params["target"].string_value();

    std::shared_ptr<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    std::shared_ptr<PointcloudEntitydata> entityData = std::dynamic_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
    if(entityData == nullptr)
    {
        log_error("Wrong type");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    upnsPointcloud2Ptr pc2 = entityData->getData();

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcdxyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(*pc2, *pcdxyz);
    ne.setInputCloud(pcdxyz);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal> cloud_normals;

    if(useK)
    {
        log_info("Using k=" + std::to_string(k) + " neighbors.");
        ne.setKSearch(k);
    }
    else
    {
        log_info("Using neighbors in sphere of radius=" + std::to_string(radius) + ".");
        ne.setRadiusSearch(radius);
    }

    // Compute the features
    ne.compute (cloud_normals);

    std::shared_ptr<pcl::PCLPointCloud2> outp(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2 cloud_normals2;
    pcl::toPCLPointCloud2(cloud_normals, cloud_normals2);
    pcl::concatenateFields(*pc2, cloud_normals2, *outp);

//    std::stringstream strstr;
//    strstr << "got normals";
//    log_info( strstr.str() );

    entityData->setData(outp);

//    OperationDescription out;
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
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "use pcl normalestimation on a pointcloud", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, &operate)
