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
     *
     *  optional <string>"frame_id" : ..., // all data given to ICP will be in this frame
     *  optional <bool>"use-metascan" : ..., //
     *
     *  <enum-as-string>"handle-result" : ["tf-add", "tf-combine", "data-change"],
     *  optional <string>"tf-frame_id" : ..., // in case of tf change or add
     *  optional <string>"tf-child_frame_id" : ..., // in case of tf change or add
     *  optional <bool>"tf-is_static" : ..., // in case of tf change or add (true only works with one input specified) (default false)
     * }
     *
     * comments:
     * - tf-add:      will add the results of the ICP without checking anything
     *                (old transforms (between "tf-frame_id" and "tf-child_frame_id")
     *                should probably been removed beforehand)
     * - tf-combine:  will combine the transforms in the system with the transforms
     *                from ICP but only at the times of ICP.
     *                That means afterwards the transforms between "tf-frame_id"
     *                and "tf-child_frame_id" are changed. In the time between the
     *                pointclouds given in the parameter "input" there will only
     *                be transforms at the time of each cloud (even so there was
     *                more transforms beforehand) these transforms will be the
     *                combination of the result of ICP and the previous transform
     *                at this time. Before and after the time of the pointclouds
     *                from "input" there will be no change.
     *                TODO: what whill happen at the gap at the start or end of
     *                      the time of the pointclouds from "input"?
     *                      E.g. there is a transform at the times [0, 10, 20, 30,
     *                      40, 50], the stampes of the pointclouds in "input" are
     *                      [19, 20, 31]. The easy way (currently implemented)
     *                      would be to have tfs at the time [0, 10, 19, 20, 31,
     *                      40, 50] afterwards, where [0, 10, 40, 50] are not
     *                      changed at all and [19, 20, 31] are the combination
     *                      of the previous transform and ICP result.
     *                      However that would mean that in the time from 10-19
     *                      and 31-40 the transform would slowly change from the
     *                      adapted ICP result to the transforms that where in
     *                      the system beforehand.
     *                      Or we could try to get a hard gap, by creating two
     *                      other transform at 19- and 31+ of the previous
     *                      transform? Then we would have the old transforms from
     *                      0-19- and 31+-50 and the new transform from 19-31.
     * - data-change: will change the data of the "input" clouds, tfs are not changed.
     */

    upns::StatusCode status;
    mapit::ICP icp(env, status);
    if ( ! upnsIsOk( status ) ) {
        return status;
    }
    return icp.operate();
}

UPNS_MODULE(OPERATOR_NAME, "execute icp on pointclouds", "fhac", OPERATOR_VERSION, "any", &operate_reg_local_icp)
