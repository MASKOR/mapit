#include <upns/operators/module.h>
#include <upns/errorcodes.h>

upns::StatusCode operateMovingLeastSquares(upns::OperationEnvironment* environment)
{
    return UPNS_STATUS_OK;
}

UPNS_MODULE(OPERATOR_NAME,
            "surface: moving least squares",
            "Marcus Meeßen",
            OPERATOR_VERSION,
            PointcloudEntitydata_TYPENAME,
            &operateMovingLeastSquares)
