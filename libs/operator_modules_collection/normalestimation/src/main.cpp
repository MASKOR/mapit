#include <upns/operators/module.h>
#include "libs/layertypes_collection/pointcloud2/include/pointcloudlayer.h"
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <iostream>
#include <pcl/features/normal_3d.h>
#include <memory>
#include <upns/errorcodes.h>
#include <upns/operators/versioning/checkoutraw.h>
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

    double radius = params["radius"].number_value();
    if(radius == 0.0)
    {
        radius = 0.5f;
    }

    std::string target = params["target"].string_value();

    upnsSharedPointer<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    upnsSharedPointer<PointcloudEntitydata> entityData = upns::static_pointer_cast<PointcloudEntitydata>( abstractEntitydata );
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

    ne.setRadiusSearch(radius);

    // Compute the features
    ne.compute (cloud_normals);

    upns::upnsSharedPointer<pcl::PCLPointCloud2> outp(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2 cloud_normals2;
    pcl::toPCLPointCloud2(cloud_normals, cloud_normals2);
    pcl::concatenateFields(*pc2, cloud_normals2, *outp);

    std::cout << "FIELDS: " << outp->fields.size() << std::endl;
    std::cout << "FIELDS2: " << cloud_normals2.fields.size() << std::endl;
    std::cout << "FIELDS1: " << pc2->fields.size() << std::endl;
//    std::stringstream strstr;
//    strstr << "got normals";
//    log_info( strstr.str() );

    entityData->setData(outp);

    OperationDescription out;
    out.set_operatorname(OPERATOR_NAME);
    out.set_operatorversion(OPERATOR_VERSION);
//    OperationParameter *outTarget = out.add_params();
//    outTarget->set_key("target");
////    outTarget->set_mapval( map->id() );
////    outTarget->set_layerval( layer->id() );
////    outTarget->set_entityval( entity->id() );
//    OperationParameter *outMapname = out.add_params();
//    outMapname->set_key("leafsize");
//    outMapname->set_realval( leafSize );
    env->setOutputDescription( out.SerializeAsString() );
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "use pcl normalestimation on a pointcloud", "fhac", OPERATOR_VERSION, PointcloudEntitydata_TYPENAME, &operate)
