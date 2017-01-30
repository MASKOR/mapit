#include "module.h"
#include "modules/versioning/checkoutraw.h"
#include "modules/operationenvironment.h"
#include <iostream>
#include <memory>
#include "upns_errorcodes.h"
#include "modules/versioning/checkoutraw.h"

upns::StatusCode operate(upns::OperationEnvironment* env)
{
    assert(false); // Not yet implemented
    upns::OperationDescription out;
    out.set_operatorname(OPERATOR_NAME);
    out.set_operatorversion(OPERATOR_VERSION);
    env->setOutputDescription( out.SerializeAsString() );
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME, "just write raw data", "fhac", OPERATOR_VERSION, "any", &operate)
