#include "module.h"
#include "libs/layertypes_collection/tf/include/tflayer.h"
#include "modules/versioning/checkoutraw.h"
#include "modules/operationenvironment.h"
#include <iostream>
#include <memory>
#include "upns_errorcodes.h"
#include "modules/versioning/checkoutraw.h"
#include "json11.hpp"

// Apply array of transforms.
// {
//    target: "path/to/target.tf"
//    mode: "relative"|"absolute",
//    tf: [
//      0: {mat: [0: m00, 1: m01, m02...], parent: "/path/to/parent", timestamp: unixts},
//      1: {mat: [0: m00_2, ...]},
//      2: ...
//    ]
// }
// relative:
// parent may be a tf or a tree
// if parent is a tree, timestamp is used to choose a appropiate coordinate system (lerp between two tfs).
// if parent is empty, target is used.
// absolute:
// only one tf (index:0) may be given, parent/timestamp must not be used.

template<typename Indexable>
void jsonToMat(float* d, Indexable &json)
{
    //TODO: unroll
    for(int i=0 ; i<16 ; ++i)
    {
        if(json[i].type() == json11::Json::NUMBER)
        {
            d[i] = static_cast<float>(json[i].number_value());
        }
    }
}

upns::StatusCode operate(upns::OperationEnvironment* env)
{
    std::string jsonErr;
    json11::Json params = json11::Json::parse(env->getParameters(), jsonErr);
    if ( ! jsonErr.empty() ) {
        // can't parth json
        // TODO: good error msg
        return UPNS_STATUS_INVALID_ARGUMENT;
    }

    std::string target = params["target"].string_value();

    bool newlyCreated = false; // In this case, there will be no tf to read
    // Get Target
    upnsSharedPointer<Entity> tfEntity = env->getCheckout()->getEntity(target);
    if(tfEntity == NULL)
    {
        // If target could not be received, create new entity
        tfEntity = upnsSharedPointer<Entity>(new Entity);
        tfEntity->set_type(TF);
        StatusCode s = env->getCheckout()->storeEntity(target, tfEntity);
        if(!upnsIsOk(s))
        {
            log_error("Failed to create entity.");
            return UPNS_STATUS_ERR_UNKNOWN;
        }
        newlyCreated = true;
    }
    upnsSharedPointer<AbstractEntitydata> abstractEntitydata = env->getCheckout()->getEntitydataForReadWrite( target );
    if(abstractEntitydata == NULL)
    {
        return UPNS_STATUS_ERR_UNKNOWN;
    }
    upnsSharedPointer<TfEntitydata> entityData = upns::static_pointer_cast<TfEntitydata>( abstractEntitydata );
    TfMatPtr tf;
    if(newlyCreated)
    {
        tf = TfMatPtr(new TfMat);
    }
    else
    {
        tf = entityData->getData();
    }

    std::string mode = params["mode"].string_value();

    if(mode == "absolute")
    {
        TfMat mat;
        *tf = TfMat::Identity();
        float *d = tf->data();
        if(params["tf"].type() == json11::Json::OBJECT)
        {
            json11::Json::object obj( params["tf"].object_items() );
            if(obj["mat"].type() == json11::Json::ARRAY)
            {
                json11::Json::array idxbl(obj["mat"].array_items());
                jsonToMat(d, idxbl);
            }
            else
            {
                log_error("wrong parameter given, absolute tf has no mat");
                return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
            }
        }
        else if(params["tf"].type() == json11::Json::ARRAY && params["tf"].array_items()[0].type() == json11::Json::OBJECT)
        {
            json11::Json::array arr( params["tf"].array_items() );
            json11::Json::object obj( arr[0].object_items() );
            if(obj["mat"].type() == json11::Json::ARRAY)
            {
                json11::Json::array idxbl(obj["mat"].array_items());
                jsonToMat(d, idxbl);
            }
            else
            {
                log_error("wrong parameter given, absolute tf[0] has no mat");
                return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
            }
        }
        else
        {
            log_error("wrong parameter given, tf is invalid");
            return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
        }
    }
    else if(mode == "relative")
    {
    }
    else
    {
        log_error("wrong parameter given, mode was not relative or absolute");
        return UPNS_STATUS_ERR_DB_INVALID_ARGUMENT;
    }
    entityData->setData(tf);


    OperationDescription out;
    out.set_operatorname(OPERATOR_NAME);
    out.set_operatorversion(OPERATOR_VERSION);
    env->setOutputDescription( out.SerializeAsString() );
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "use pcl normalestimation on a pointcloud", "fhac", OPERATOR_VERSION, upns::LayerType::POINTCLOUD, &operate)
