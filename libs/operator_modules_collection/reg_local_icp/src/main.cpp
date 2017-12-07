#include <upns/operators/module.h>
#include <upns/logging.h>
#include <upns/operators/versioning/checkoutraw.h>
#include <upns/operators/operationenvironment.h>
#include <upns/errorcodes.h>
#include <string>

#include <upns/layertypes/tflayer.h>
#include <upns/layertypes/tflayer/utils.h>
#include <upns/layertypes/tflayer/tf2/buffer_core.h>

#include "icp.h"

upns::StatusCode operate_reg_local_icp(upns::OperationEnvironment* env)
{
    /** structure:
     * {
     *  <string>"tf-prefix" : ..., // the prefix where to look for transforms (default "")
     *  <string>"input"[] : ..., // can either be, a list of entities or a tree containing enteties
     *  <string>"target" : ...,
     *  optional <string>"frame_id" : ..., // all data given to ICP will be in this frame
     *  <enum-as-string>"handle-result" : ["tf-add", "tf-change", "data-change"],
     *  optional <string>"tf-frame_id" : ..., // in case of tf change or add
     *  optional <string>"tf-child_frame_id" : ..., // in case of tf change or add
     *  optional <bool>"tf-is_static" : ..., // in case of tf change or add (true only works with one input specified) (default false)
     * }
     */

    upns::StatusCode status;
    mapit::ICP icp(env, status);
    if ( ! upnsIsOk( status ) ) {
        return status;
    }
    return icp.operate();
}

UPNS_MODULE(OPERATOR_NAME, "execute icp on pointclouds", "fhac", OPERATOR_VERSION, "any", &operate_reg_local_icp)
