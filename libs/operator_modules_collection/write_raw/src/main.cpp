#include "module.h"
#include "libs/layertypes_collection/tf/include/tflayer.h"
#include "modules/versioning/checkoutraw.h"
#include "modules/operationenvironment.h"
#include <iostream>
#include <memory>
#include "error.h"
#include "modules/versioning/checkoutraw.h"

upns::StatusCode operate(upns::OperationEnvironment* env)
{
    assert(false); // Not yet implemented
    OperationDescription out;
    out.set_operatorname(OPERATOR_NAME);
    out.set_operatorversion(OPERATOR_VERSION);
    env->setOutputDescription( out.SerializeAsString() );
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "just write raw data", "fhac", OPERATOR_VERSION, upns::LayerType::ANY_RAW, &operate)
